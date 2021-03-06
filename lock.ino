#include <limits.h>
#include <stdint.h>
#include <SPI.h>
#include <RF24.h>

// #define PRO_MINI
#define MICRO

/*
 * Set the following to 1 to run debug logic and output debug information to the
 * serial output.
 */
#define DEBUG 0

// RF24 RADIO
#define RADIO_CHANNEL 5

// ANALOG PINS
#define PIN_POWER_LEVEL 4
#define PIN_MOTOR_FEEDBACK 5

// DIGITAL PINS
#define PIN_MOTOR_UNLOCK 4
#define PIN_MOTOR_LOCK 5
#define PIN_BUTTON1 6
#define PIN_BUTTON2 7
#define PIN_MASTER_JUMPER 8
#define PIN_LIGHT1 9
#define PIN_LIGHT2 10

// POWER THRESHOLDS
#define THRESHOLD_AC 800
#define THRESHOLD_BATTERY 550
#define THRESHOLD_LOCK 550
#define THRESHOLD_UNLOCK 50

// SYNCHRONIZATION
#define HEARTBEAT_TIMEOUT 1000
#define HEARTBEAT 500

#define TRACE( x ) do { if ( DEBUG ) { x } } while ( 0 )
#define PRINT( x ) ( Serial.print( x ) )
#define PRINTLN( x ) ( Serial.println( x ) )

/* -------------------------------------------------------------------------- */

/*
 * Represents a linked list of communication devices (Arduinos with radios).
 *
 * device_id - represents the unique communication id of the radio device.
 * seq_num   - represents a unique and increasing number across consecutive
 *             transmission of packets for this device. This value should be
 *             updated if a packet with a larger seq_num is received for this
 *             device_id. If a packet is received with a smaller seq_num, then
 *             it should be ignored.
 * next      - represents the next device in the the linked list of devices that
 *             have transmitted packets. NULL if there is no such next device.
 */
typedef struct device_t {
  unsigned long id;
  unsigned long seq_num;
  device_t* next;
} device_t;

/*
 * Represents the type of data being transmitted in a packet_t. The MASTER only
 * transmits SYNC packets. Only clients transmit EVENT packets.
 */
typedef enum {
  EVENT = 1,
  SYNC = 2
} packet_type_t;

/*
 * Data structure used for communicating with other radio devices.
 *
 * device_id - represents the unique communication id of a radio device.
 * seq_num   - represents a unique and increasing number across consecutive
 *             transmission of packets for this device.
 * type      - represents the type of information carried by the packet. the
 *             locked field is only used for SYNC packets.
 * locked    - represents the current lock state for SYNC packets.
 */
typedef struct {
  unsigned long device_id;
  unsigned long seq_num;
  packet_type_t type;
  bool locked;
} packet_t;

class Button {

    enum State {
      PRESSED = 1,
      DOWN = 2,
      UP = 3,
      RELEASED = 4
    };
  
    const int pin;
    State state;
    
  public:

    Button( int pin ): pin( pin ), state( UP ) {
      pinMode( pin, INPUT_PULLUP );
      update();
    }
    
    void update() {
      bool up = digitalRead( pin );
      if ( up ) {
        if ( state == DOWN || state == PRESSED ) {
          state = RELEASED;
        } else {
          state = UP;
        }
      } else {
        if ( state == UP || state == RELEASED ) {
          state = PRESSED;
        } else {
          state = DOWN;
        }
      }
    }

    bool isUp() {
      return state == UP;
    }

    bool isDown() {
      return state == DOWN;
    }

    bool isPressed() {
      return state == PRESSED;
    }

    bool isReleased() {
      return state == RELEASED;
    }
    
};

class Light {
  
    const int pin;
    bool on;
    
  public:

    Light( int pin ): pin( pin ), on( false ) {
      pinMode( pin, OUTPUT );
      update();
    }

    void update() {
      digitalWrite( pin, on );
    }

    bool isOn() {
      return on;
    }

    bool isOff() {
      return !on;
    }

    void turnOn() {
      on = true;
    }

    void turnOff() {
      on = false;
    }
    
};

class Motor {
  
    enum State {
      NEUTRAL = 0,
      UNLOCKING = 1,
      LOCKING = 2,
      STOPPED = 3
    };

    const int lock_pin;
    const int unlock_pin;
    const int feedback_pin;
    int position;
    State state;

  public:

    Motor( int lock_pin, int unlock_pin, int feedback_pin ): lock_pin( lock_pin ),
                                                             unlock_pin( unlock_pin ),
                                                             feedback_pin( feedback_pin ),
                                                             state( STOPPED ) {
      pinMode( lock_pin, OUTPUT );
      pinMode( unlock_pin, OUTPUT );
      update();
    }

    void update() {
      position = analogRead( feedback_pin );
      digitalWrite( lock_pin, state & LOCKING );
      digitalWrite( unlock_pin, state & UNLOCKING );
    }

    int getPosition() {
      return position;
    }

    bool isNeutral() {
      return state == NEUTRAL;
    }

    bool isUnlocking() {
      return state == UNLOCKING;
    }

    bool isLocking() {
      return state == LOCKING;
    }

    bool isStopped() {
      return state == STOPPED;
    }

    void neutral() {
      state = NEUTRAL;
    }

    void unlock() {
      state = UNLOCKING;
    }

    void lock() {
      state = LOCKING;
    }

    void stop() {
      state = STOPPED;
    }
  
};

class Lock {

    Motor * const motor;
    unsigned int engaged;

  public:

    Lock( Motor *m ): motor( m ), engaged( 0 ) {
      update();
    }

    void update() {
      if ( engaged ) {
        if ( motor->getPosition() < THRESHOLD_LOCK ) {
          motor->lock();
        } else {
          motor->stop();
        }
      } else {
        if ( motor->getPosition() > THRESHOLD_UNLOCK ) {
          motor->unlock();
        } else {
          motor->stop();
        }
      }
    }

    bool isEngaged() {
      return engaged;
    }

    bool get( unsigned int i ) {
      return ( engaged >> i ) & 1U;
    }

    void set( unsigned int i, bool lock ) {
      if ( lock ) engaged |= 1U << i;
      else engaged &= ~( 1U << i );
    }

    bool toggle( unsigned int i ) {
      engaged ^= 1U << i;
      return get( i );
    }

    void clear() {
      engaged = 0;
    }
  
};

//////////////////////////////////////////////////////////////////////////

bool MASTER = false;
bool low_power = false;
int power_level;
int heartbeat_timeout;
int heartbeat;

// Variables used for internal state
Button *button1, *button2;
Light *light1, *light2;
Motor *motor;
Lock *lock;

// Variables used for communication
unsigned long device_id;
unsigned long seq_num;
device_t* devices;
packet_t packet;

// Variables for radio
#ifdef PRO_MINI
RF24 radio( 9, 10 );
#elif MICRO
RF24 radio( 2, 3 );
#endif
const uint64_t pipes[2] = { 0xe7e7e7e7e7LL, 0xc2c2c2c2c2LL };

//////////////////////////////////////////////////////////////////////////

inline void updatePowerLevel() {
  power_level = analogRead( PIN_POWER_LEVEL );
}

inline device_t* addDevice( unsigned long id ) {

  TRACE( PRINTLN( "Adding device." ); );
  
  device_t* d = ( device_t* ) malloc( sizeof( device_t ) );
  
  d->id = id;
  d->seq_num = 0;
  d->next = devices;
  devices = d;
  
  return d;
  
}

inline device_t* getDevice( unsigned long id ) {
  
  if ( !id ) return NULL;
  
  device_t* curr = devices;
  while ( curr ) {
    if ( curr->id == id ) return curr;
    curr = curr->next;
  }
  
  return addDevice( id );
  
}

inline void printPacket() {
  PRINT( "device_id: " );
  PRINTLN( packet.device_id );
  PRINT( "seq_num: " );
  PRINTLN( packet.seq_num );
  PRINT( "type: " );
  PRINTLN( packet.type );
  PRINT( "locked: " );
  PRINTLN( packet.locked );
}

inline void xmitPacket() {

  TRACE(
    PRINTLN( "Sending packet." );
    printPacket();
  );

  // Switch to xmit mode
  radio.openWritingPipe( pipes[ 1 ] );
  radio.openReadingPipe( 1, pipes[ 0 ] );
  radio.stopListening();

  // Send packet
  radio.write( &packet, sizeof( packet_t ) );

  // Switch back to recv mode
  radio.openWritingPipe( pipes[ 0 ] );
  radio.openReadingPipe( 1, pipes[ 1 ] );
  radio.startListening();
  
}

inline bool recvPacket() {
  if ( radio.available() ) {

    // Get packet
    radio.read( &packet, sizeof( packet_t ) );

    TRACE( PRINTLN( "Received packet." ); );

    // Get associated device
    device_t* device = getDevice( packet.device_id );

    // Only accept new packets from valid devices
    if ( device && packet.seq_num > device->seq_num ) {
      TRACE( printPacket(); );
      device->seq_num = packet.seq_num;
      return true;
    }
    
  }
  return false;
}

//////////////////////////////////////////////////////////////////////////

void setup() {
  
  TRACE(
    Serial.begin( 9600 );
    delay( 2000 );
  );

  randomSeed( analogRead( 0 ) );

  pinMode( PIN_MASTER_JUMPER, INPUT_PULLUP );

  MASTER = !digitalRead( PIN_MASTER_JUMPER );

  device_id = random( LONG_MAX );
  seq_num = random( 1L << ( sizeof( seq_num ) * 4 ) );

  heartbeat_timeout = 0;
  heartbeat = HEARTBEAT;

  button1 = new Button( PIN_BUTTON1 );
  button2 = new Button( PIN_BUTTON2 );
  light1 = new Light( PIN_LIGHT1 );
  light2 = new Light( PIN_LIGHT2 );
  motor = new Motor( PIN_MOTOR_LOCK, PIN_MOTOR_UNLOCK, PIN_MOTOR_FEEDBACK );
  lock = new Lock( motor );

  radio.begin();
  radio.setChannel( RADIO_CHANNEL );
  radio.setAutoAck( 1 );
  radio.setRetries( 5, 10 );
  radio.setPayloadSize( sizeof( packet_t ) );
  radio.setCRCLength( RF24_CRC_8 );
  radio.setDataRate( RF24_250KBPS );
  radio.setPALevel( RF24_PA_MAX );
  radio.openWritingPipe( pipes[ 0 ] );
  radio.openReadingPipe( 1, pipes[ 1 ] );
  radio.startListening();
  
}

//////////////////////////////////////////////////////////////////////////

void loop() {

  updatePowerLevel();
  button1->update();
  button2->update();
  light1->update();
  light2->update();
  motor->update();
  lock->update();

  if ( lock->get( 0 ) ) {
    light1->turnOn();
  } else {
    light1->turnOff();
  }

  if ( lock->get( 1 ) ) {
    light2->turnOn();
  } else {
    light2->turnOff();
  }
  
  if ( low_power ) {
    if ( power_level > THRESHOLD_AC ) {
      TRACE( PRINTLN( "LEAVING LOW POWER MODE." ); );
      low_power = false;
    } else {
      TRACE( PRINTLN( "LOW POWER MODE." ); );
      lock->clear();
      delay( 1 );
      return;
    }
  } else if ( power_level < THRESHOLD_BATTERY ) {
    
    TRACE( PRINTLN( "ENTERING LOW POWER MODE." ); );
    
    low_power = true;
    
    // If a master dies (i.e. low power), the clients will automatically follow suit
    // However, if a client dies, we need to tell master
    if ( !MASTER && lock->isEngaged() ) {
      packet.device_id = device_id;
      packet.seq_num = seq_num++;
      packet.type = EVENT;
      xmitPacket();
    }
    
    delay( 1 );
    return;
    
  }
  
  if ( MASTER ) {
    if ( heartbeat ) {
      heartbeat--;
    } else {

      TRACE( PRINTLN( "Sending heartbeat sync." ); );

      // Send heartbeat sync packet
      packet.device_id = device_id;
      packet.seq_num = seq_num++;
      packet.type = SYNC;
      packet.locked = lock->get( 0 );
      
      xmitPacket();

      // Reset the counter
      heartbeat = HEARTBEAT;
      
    }
  } else if ( heartbeat_timeout == HEARTBEAT_TIMEOUT ) {

    // We haven't heard from the server for long enough; unlock for safety
    lock->clear();

    // Reset the counter
    heartbeat_timeout = 0;

    delay( 1 );
    return;
    
  } else {
    heartbeat_timeout++;
  }

  if ( recvPacket() ) {
    if ( packet.type == EVENT && MASTER ) {

      // MASTER updates internal state and tells clients to SYNC to new state
      packet.device_id = device_id;
      packet.seq_num = seq_num++;
      packet.type = SYNC;
      packet.locked = lock->toggle( 0 );
      
      xmitPacket();
      
    } else if ( packet.type == SYNC ) {

      lock->set( 0, packet.locked );

      // Reset the heartbeat
      heartbeat = HEARTBEAT;
      heartbeat_timeout = 0;
      
    }
  }

  if ( button1->isReleased() ) {
    
    packet.device_id = device_id;
    packet.seq_num = seq_num++;

    if ( MASTER ) {

      // MASTER updates internal state and tells clients to SYNC to new state
      packet.type = SYNC;
      packet.locked = lock->toggle( 0 );
      
    } else {

      // Clients only tell MASTER about event
      packet.type = EVENT;
      
    }
    
    xmitPacket();
    
  }

  if ( MASTER && button2->isReleased() ) {
    lock->toggle( 1 );
  }

  delay( 1 );

}
