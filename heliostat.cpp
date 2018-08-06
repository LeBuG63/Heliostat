#include <Wire.h>
#include <Servo.h>
#include "LiquidCrystal_I2C.h"

#define RESI_NIGHT          550   // 550 (0 - 1023)
#define WIND_SPEED_WARNING  20.0f // km/h
#define DELAY_NO_WIND       200  // ms

#define TIMER_UPDATE_AUTO   1000 // ms
#define TIMER_UPDATE_MANUAL 300 // ms

#define ARCTAN( x )   ( atan( x ) / PI * 180 )
#define ARCCOS( x )   ( acos( x ) / PI * 180 )
#define COS( x )      ( cos( x ) )
#define SIN( x )      ( sin( x ) )

LiquidCrystal_I2C lcd( 0x20, 20, 4 );

/*
   1 - NORD
   2 - SUD
   3 - EST
   4 - OUEST
*/

enum {
  g_PinAnemometre = 7,
  g_PinSwitchManualMode = 4,
  g_PinAPotAzymut = A0,
  g_PinAPotElev = A1
} PIN;

int           g_mode;

unsigned long g_startTimerUpdate;
unsigned long g_timerUpdate;

String int_to_stringMode[ ] = {
  "Auto",
  "Manuel",
  "Nuit",
  "Arret (Vent)"
};

void updateModeSwitchButton( ) {
  g_mode = digitalRead( g_PinSwitchManualMode );
}

typedef enum {
  AUTOMATIC,
  MANUAL,
  NIGHT,
  WIND
} mode_t;

typedef enum {
  RIGHT,
  LEFT
} dir_t;

typedef enum {
  WEST,
  EAST,
  NORTH,
  SOUTH,
  NOTHING
} axe_dir_t;

class hlsServoMot {
  public:
    hlsServoMot( )
    {}

    void Set( const int pin, const int minAngle, const int zeroAngle, const int maxAngle ) {
      m_minAngle = minAngle;
      m_maxAngle = maxAngle;

      m_zeroAngle = zeroAngle;
      m_absAngle = m_minAngle;

      m_servo.attach( pin );

      SetRotationAngle( zeroAngle );
      Turn( RIGHT );
      SetRotationAngle( 0 );
    }

    int SetRotationAngle( int val ) {
      _SetAngleRotation( &val );

      m_addAngle = val;
    }

    void SetPosFromPot( const int pin ) {
      static int last = 0;

      int potValue = analogRead( pin );
      int angle = map( potValue, 0, 1023, m_minAngle, m_maxAngle );

      m_absAngle = angle;

      if ( last >= angle + 1 || last <= angle - 1 )
        m_servo.write( angle );

      last = angle;
    }

    void Turn( const dir_t dir ) {
      if ( m_addAngle != 0 ) {
        int angle;

        angle = ( dir == RIGHT ) ? m_addAngle : -m_addAngle;

        _SetAngleRotation( &angle );

        m_absAngle += angle;
        m_servo.write( m_absAngle );
      }
    }

    void TurnFromAngle( const int angle ) {
      m_servo.write( angle );
    }

    void TurnFromPyramid( const axe_dir_t dir) {
      if ( dir == WEST || dir == NORTH ) {
        Turn( LEFT );
      } else if ( dir == EAST || dir == SOUTH ) {
        Turn( RIGHT );
      }
    }

    void Laydown( ) {
      TurnFromAngle( 0 );
    }

    int GetAbsAngle( ) {
      return m_absAngle;
    }

    int GetAngle( ) {
      return m_absAngle - m_minAngle;
    }

    int GetPos( ) {
      return m_servo.read( );
    }

  private:
    void _SetAngleRotation( int *val ) {
      int totAngle = m_absAngle + *val;

      if ( totAngle > m_maxAngle ) {
        *val = -m_absAngle + m_minAngle;
      }
      else if ( totAngle < m_minAngle ) {
        *val = -m_absAngle + m_maxAngle;
      }
    }

  private:
    Servo m_servo;

    int   m_absAngle;
    int   m_addAngle;

    int   m_minAngle;
    int   m_maxAngle;

    int   m_zeroAngle;
};

class Pyramid {
  public:
    Pyramid( ) {};

    void Set( const int pin_e, const int pin_w, const int pin_n, const int pin_s ) {
      pinMode( pin_e, INPUT );
      pinMode( pin_w, INPUT );
      pinMode( pin_n, INPUT );
      pinMode( pin_s, INPUT );

      m_pinAxe[ EAST ] = pin_e;
      m_pinAxe[ WEST ] = pin_w;
      m_pinAxe[ SOUTH ] = pin_s;
      m_pinAxe[ NORTH ] = pin_n;
    }

    int GetAxeValue( const axe_dir_t dir ) {
      return m_valueAxe[ dir ];
    }

    void GetAxesValues( )  {
      m_valueAxe[ EAST ] = analogRead( m_pinAxe[ EAST ] );
      m_valueAxe[ WEST ] = analogRead( m_pinAxe[ WEST ] );
      m_valueAxe[ SOUTH ] = analogRead( m_pinAxe[ SOUTH ] );
      m_valueAxe[ NORTH ] = analogRead( m_pinAxe[ NORTH ] );
    }

    axe_dir_t AxeToAimSunAzy( ) {
      int v_east = GetAxeValue( EAST );
      int v_west = GetAxeValue( WEST );

      int p = ( ( float )  v_east / ( float )  v_west ) * 1000;

      if ( p < ( 1000 - m_azyOffset ) )
        return EAST;
      else if ( p > ( 1000 + m_azyOffset ) )
        return WEST;
      else
        return NOTHING;
    }

    axe_dir_t AxeToAimSunEle( ) {
      int v_north = GetAxeValue( NORTH );
      int v_south = GetAxeValue( SOUTH );

      int p = ( ( float )  v_north / ( float )  v_south ) * 1000;

      if ( p < ( 1000 - m_eleOffset ) )
        return NORTH;
      else if ( p > ( 1000 + m_eleOffset ) )
        return SOUTH;
      else
        return NOTHING;
    }

    bool CheckNight( ) {
      int v_axes = GetAxeValue( SOUTH ) + GetAxeValue( NORTH ) + GetAxeValue( EAST ) + GetAxeValue( WEST );

      if ( ( v_axes / 4 )   < RESI_NIGHT ) {
        g_mode = NIGHT;
        return true;
      }
      return false;
    }

  private:
    int m_valueAxe[ 4 ];
    int m_pinAxe[ 4 ];

    int m_azyOffset = 100;
    int m_eleOffset = 100;
};

int MotAzyAimPoint( const int o, const int s ) {
  float _o = (float)  o / 180 * PI;
  float _s = (float)  s / 180 * PI;

  return ( int )  abs( ( ARCTAN( ( SIN( _o ) * COS( _s ) ) / ( 1 + COS( _o ) * COS( _s ) ) ) ) );
}

int MotEleAimPoint( const int o, const int s ) {
  float _o = (float) o / 180 * PI;
  float _s = (float) s / 180 * PI;

  return ( int )  ( 90.0f - ARCCOS( sin( _s )   / sqrt( pow( ( 1 + COS( _o ) * COS( _s ) )  , 2 ) + pow( ( SIN( _o ) * COS( _s ) )  , 2 ) + ( 1 - COS( 2 * _s ) ) ) ) );
}

unsigned _pulseIn( const int pin, const bool state ) {
  bool pinState;

  pinState = digitalRead( pin );

  if ( pinState == state ) {
    unsigned long timerStart;
    unsigned long timerEnd;
    unsigned long deltaTime;

    timerStart = millis( );

    while ( digitalRead( pin ) == state )
      if ( ( millis( ) - timerStart ) >= DELAY_NO_WIND )
        return 0;

    timerEnd = millis( );

    deltaTime = timerEnd - timerStart;

    return deltaTime;
  }

  return 0;
}

float GetMoySpeedWind( ) {
  int n_value = 10;
  float moy = 0.0;

  float V;
  float P;

  for ( int i = 0; i < n_value; ++i ) {
    unsigned haut = _pulseIn( g_PinAnemometre , HIGH );
    unsigned bas = _pulseIn( g_PinAnemometre , LOW );

    if ( haut == 0 && bas == 0 ) {
      return 0.0f;
    }

    P = ( haut + bas ) * 2;

    V = ( ( 1 / P ) * 0.44 ) * 3.6 * 1000 * 1.159;

    moy += V;
  }
  
  moy = moy / n_value;

  return moy;
}

////////////////////////////////
hlsServoMot g_servoAzy, g_servoMirrorAzy;
hlsServoMot g_servoEle, g_servoMirrorEle;
Pyramid     g_pyramid;
////////////////////////////////

void PrintInfo( ) {
  String stringMode;
  String stringAzy;
  String stringEle;
  String stringWind;
  
  stringWind = "Vent:   ";

  if ( g_mode == MANUAL ) {
    stringWind += "Anemo arret";
  } else {
    float speedWind = GetMoySpeedWind( );
    stringWind += String( speedWind ) + "km/h    ";

    if ( static_cast< int > ( speedWind ) > static_cast < int > ( WIND_SPEED_WARNING ) ) {
      g_mode = WIND;
    }
  }

  stringMode = "Mode:   " +         int_to_stringMode[ g_mode ] +                    "            ";
  stringAzy  = "Azimut: " + String( g_servoAzy.GetAngle( ) )    + static_cast< char >( 223 ) + "    ";
  stringEle  = "Elev:   " + String( g_servoEle.GetAbsAngle( ) ) + static_cast< char >( 223 ) + "    ";


  lcd.setCursor( 0, 0 );
  lcd.print( stringMode );

  lcd.setCursor( 0, 1 );
  lcd.print( stringAzy );

  lcd.setCursor( 0, 2 );
  lcd.print( stringEle );

  lcd.setCursor( 0, 3 );
  lcd.print( stringWind );
}

void Update( void ) {
  static int tmpValAzy = 0;
  static int tmpValEle = 0;

  updateModeSwitchButton( );

  if ( ( millis( ) - g_startTimerUpdate ) > g_timerUpdate ) {
    int azyAngle = g_servoAzy.GetAbsAngle( );
    int eleAngle = g_servoEle.GetAbsAngle( );

    int motAzyPoint = MotAzyAimPoint( azyAngle, eleAngle );
    int motElePoint = MotEleAimPoint( azyAngle, eleAngle );

    if ( motAzyPoint != tmpValAzy ) {
      tmpValAzy = motAzyPoint;
      g_servoMirrorAzy.TurnFromAngle( motAzyPoint );
    }
    if ( tmpValEle != tmpValEle ) {
      tmpValEle = motElePoint;
      g_servoMirrorEle.TurnFromAngle( motElePoint );
    }
    
    PrintInfo( );

    g_startTimerUpdate = millis( );
  }
  else
    delay( 10 );
}

void setup( ) {
  Serial.begin( 9600 );

  pinMode( g_PinSwitchManualMode, INPUT );
  pinMode( g_PinAPotAzymut, INPUT );
  pinMode( g_PinAPotElev, INPUT );

  lcd.init( );
  lcd.backlight( );
  lcd.begin( 20, 4 );

  g_servoAzy.Set( 13, 0, 0, 180 );
  g_servoAzy.SetRotationAngle( 1 );

  g_servoEle.Set( 12, 0, 0, 180 );
  g_servoEle.SetRotationAngle( 1 );

  g_servoMirrorAzy.Set( 11, 0, 0, 180 );
  g_servoMirrorEle.Set( 10, 0, 0, 180 );

  g_pyramid.Set( A12, A13, A14, A15 );

  lcd.setCursor( 5, 1 );
  lcd.print( "Heliostat" );

  g_mode = AUTOMATIC;
  g_timerUpdate = TIMER_UPDATE_AUTO;

  g_startTimerUpdate = millis( );
}

void loop( ) {
  bool isNight = g_pyramid.CheckNight( );

  if ( g_mode == MANUAL && isNight )
    g_mode = MANUAL;

  switch ( g_mode ) {
    case WIND: {
        float windSpeed = GetMoySpeedWind( );

        g_servoMirrorEle.Laydown();
        
        if ( static_cast< int >( windSpeed ) < static_cast< int >( WIND_SPEED_WARNING ) )
          g_mode = AUTOMATIC;
        else
          g_mode = WIND;
      }
      break;

    case NIGHT: {
        g_servoEle.Laydown( );
        g_pyramid.GetAxesValues( );

        if ( !isNight )
          g_mode = AUTOMATIC;
      }
      break;

    case MANUAL: {
        int azyAngle = g_servoAzy.GetAbsAngle( );
        int eleAngle = g_servoEle.GetAbsAngle( );

        g_servoMirrorAzy.SetPosFromPot( g_PinAPotAzymut );
        g_servoMirrorEle.SetPosFromPot( g_PinAPotElev );

        g_timerUpdate = TIMER_UPDATE_MANUAL;
      }
      break;

    case AUTOMATIC: {
        g_pyramid.GetAxesValues( );

        g_servoAzy.TurnFromPyramid( g_pyramid.AxeToAimSunAzy( ) );
        g_servoEle.TurnFromPyramid( g_pyramid.AxeToAimSunEle( ) );

        g_timerUpdate = TIMER_UPDATE_AUTO;
      }
      break;
    default:
      break;
  }
  Update();
}
