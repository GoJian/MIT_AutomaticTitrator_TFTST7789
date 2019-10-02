// Notes on BD 10 mL Syringe Mechanical Specifications:
//
// Geometry: 10 mL, 14.5 mm ID, 1.6513 cm^2 Area
// Calibration: 1 mm Height = 0.16513 mL = 165.13 uL (165.13 uL per mm/3200 steps)
// 
// 1 step = 0.0516031 uL <-- this number is to be measured during calibration.
//
// Dynamic titrant dispense scheme (linear regression model)
// Titration Scheme:
//   0) Initialize pH of the solution to pH ~3 by adding HCl, wait for pH to stabilize (or timeout if does not stabilize).
//      (Stability is checked by computing the standard deviation of N=5 recent pH values obtained.)
//   1) Dispense TITRATE_VOL (beginning with default TITRATE_VOL_NORM).
//   2) Wait for pH to be stable again, then dispense again TITRATE_VOL.
//   3) Measure/Calculate at every TITRATE_VOL: pH_value, 
//                                              pH_1st_derivative (3 data points),
//                                              pH_2nd_derivative (3 data points),
//                                              pH_3rd_derivative (5 data points),
//                                              RMSE (Root Mean Square Error) from linear regression.
//      (Storing 5 continuous paired data points (vol and pH) at each step for computation and analysis)
//   4) Dynamically adjust TITRATE_VOL such that it is smaller (higher resolution) when the curve is nonlinear [large RMSE],
//      larger (coarser resolution) when the curve is linear [small RMSE]. Back to 2) in a loop.
//
// Calculating TITRATE_VOL:
//
// TITRATE_VOL <-- [TITRATE_VOL_MIN, TITRATE_VOL_MAX]: a pre-programmed interval
//   When the slope is high (pH change is rapid), reduce the TITRATE_VOL towards TITRATE_VOL_MIN.
//   When the slope is low (pH change is slow), increase the TITRATE_VOL towards TITRATE_VOL_MAX.
//
//   The slope is characterized by pH_1st_derivative, which has a range of [0 INF]. Therefore,
//   we want INF to map to TITRATE_VOL_MIN and 0 to map to TITRATE_VOL_MAX.
//
//   A natural analytical function that can achieve this is:
//        TITRATE_VOL (pH_1st_derivative) = (TITRATE_VOL_MAX - TITRATE_VOL_MIN) * EXP (- a * pH_1st_derivative) + TITRATE_VOL_MIN;
//   where a is a tuning variable. A larger a value means to prefer smaller steps (higher resolution) when the slope is high.
//   default a = 0.00005;

// double TUNE_A = 0.00005;
//
// data_value_tm2;
// data_value_tm1;
// data_value_t0;
// data_value_tp1;
// data_value_tp2;
// delta_t; // this is a fixed value
//
// data_1st_derivative = (data_value_tp1 - data_value_tm1)/(2*t_delta);
// data_2nd_derivative = (data_value_tp1 - data_value_t0 - data_value_t0 + data_value_tm1)/(t_delta*t_delta);
// data_3rd_derivative = (data_value_tp2 - 2*data_value_tp1 + 2*data_value_tm1 - data_value_tm2)/(2*t_delta^3);

// const unsigned long pH_wait_time = 1000*60*3;
// unsigned long timeMarker;

// void pH_wait_for_stable(unsigned long pH_wait_time) { // wait until pH is stable or timeout at 3 minutes
//   timeMarker = millis();
//   while ( (millis() - timeMarker) < pH_wait_time) {
//     if ( (pH_value(1) - pH_value(0)) <= pH_delta_stable ) break;
//   }
//   return;
// }

double pump_uL_PER_STEP = 0.0516031;  // With syringe pump operating at 3200 steps/resolution = 1 mm, a 10 mL BD syringe gives
                                      // a theoretical resolution of <this number> (uL per microstep) [3200 microsteps = 1 rev]
const double TITRATE_VOL_NORM = 25.;  // default to 25 uL per step
const double TITRATE_VOL_MIN  = 5.;   // default to 25 uL per step
const double TITRATE_VOL_MAX  = 50.; // default to 25 uL per step

const double pH_DELTA_STABLE = 0.03;  // Accepted stable pH

const double TITRATION_DELTA_T = 2;   // default stabilizing time interval (this is fixed)
const int    TITRATION_WAIT_T  = 3;   // minimum wait time after fluid dispence (minimum wait time for pH to be stable)

const int TITRATE_STEP_NORM = int( TITRATE_VOL_NORM/pump_uL_PER_STEP );
const int TITRATE_STEP_MIN  = int( TITRATE_VOL_MIN /pump_uL_PER_STEP );
const int TITRATE_STEP_MAX  = int( TITRATE_VOL_MAX /pump_uL_PER_STEP );

const double TUNE_A = 0.00005;

class dataBuffer { // circular buffer for data keeping & computation
  public:
    double *dataBufferData;
    int dataBufferPos, dataBufferLen, dataBufferFill;

    dataBuffer(int len) {
      dataBufferData=(double *)malloc( sizeof(double) * len );
      dataBufferPos  = 0;
      dataBufferLen  = len;  
      dataBufferFill = 0;
    }

    void insert(double value) {
      dataBufferData[dataBufferPos] = value;
      dataBufferPos = (dataBufferPos+1) % dataBufferLen; // circular buffer FILO logic
      dataBufferFill++;
    }

    void stepback() {
      dataBufferPos = (dataBufferPos+dataBufferLen-1) % dataBufferLen; // roll back position 1 step
    }
  
    int len() {
      return min(dataBufferFill,dataBufferLen);    
    }

    bool isFilled() {
      if (dataBufferFill >= dataBufferLen)
        return 1;
      else
        return 0;
    }

    bool isStable() {
      if (this->stdev() < pH_DELTA_STABLE )
        return 1;
      else
        return 0;
    }

    double mean() {
      double sum = 0.;
      int len = this->len();
      if( len > 0 ) {
        for(int n=0; n<len; n++) {
          sum += dataBufferData[n]; 
        }
        return sum/len;
      } else
        return 0; 
    }

    bool is_1st2nd_dev_ready() {
      if (dataBufferFill >= 3)
        return 1;
      else
        return 0;
    }

    bool is_3rd_dev_ready() {
      if (dataBufferFill >= 5)
        return 1;
      else
        return 0;
    }

    double first_dev(double t_delta) { // 1st_derivative
      double data_value_tm1 = dataBufferData[(dataBufferPos+2+dataBufferLen) % dataBufferLen]; // +2 (back in time)
      double data_value_tp1 = dataBufferData[dataBufferPos];                                   // current

      return (data_value_tp1 - data_value_tm1)/(2.*t_delta);
    }

    double second_dev(double t_delta) { // 2nd_derivative
      double data_value_tm1 = dataBufferData[(dataBufferPos+2+dataBufferLen) % dataBufferLen]; // +2
      double data_value_t0  = dataBufferData[(dataBufferPos+1+dataBufferLen) % dataBufferLen]; // +1
      double data_value_tp1 = dataBufferData[dataBufferPos];                                   // current

      return (data_value_tp1 - data_value_t0 - data_value_t0 + data_value_tm1)/(t_delta*t_delta);
    }

    double third_dev(double t_delta) { // 3rd_derivative
      double data_value_tm2 = dataBufferData[(dataBufferPos+4+dataBufferLen) % dataBufferLen]; // +4
      double data_value_tm1 = dataBufferData[(dataBufferPos+3+dataBufferLen) % dataBufferLen]; // +3
      //double data_value_t0  = dataBufferData[(dataBufferPos-2+dataBufferLen) % dataBufferLen]; // +2
      double data_value_tp1 = dataBufferData[(dataBufferPos+1+dataBufferLen) % dataBufferLen]; // +1
      double data_value_tp2 = dataBufferData[dataBufferPos];                                   // current

      return (data_value_tp2 - 2.*data_value_tp1 + 2.*data_value_tm1 - data_value_tm2)/(2.*t_delta*t_delta*t_delta);
    }

    double compute_titration_uL_Volume (double pH_max_slope) {
      double titration_uL_Volume = (TITRATE_VOL_MAX - TITRATE_VOL_MIN)*exp(-TUNE_A*pH_max_slope) + TITRATE_VOL_MIN;

      return titration_uL_Volume;
    }

    int compute_titration_steps_Volume (double pH_max_slope) {
      return int( this->compute_titration_uL_Volume(pH_max_slope) / pump_uL_PER_STEP );
    }

    double stdev() {
      double mean = this->mean();
      double sum = 0;
      int len = this->len();
      if( len > 0 ) {
        for(int n=0; n<len; n++) {
          sum += pow(dataBufferData[n]-mean,2);
        }
        return sqrt( sum / len );
      } else
        return 0;
    }

    double linearRegRMSE(dataBuffer *x, dataBuffer *y) { // population rmse against linear model
      double x_mean = x->mean();
      double y_mean = y->mean();

      double var=0.,covar=0.,a,b,rmse; // assuming y = ax + b
      int len = x->len();
      for(int n=0; n<len; n++) {
        var   += pow( x->dataBufferData[n] - x_mean, 2);
        covar += ( x->dataBufferData[n] - x_mean )*( y->dataBufferData[n] - y_mean );
      }
      a = covar / var;
      b = y_mean - a * x_mean;
      for(int n=0; n<len; n++) {
        rmse += pow( y->dataBufferData[n] - a*x->dataBufferData[n] - b, 2);
      }
      rmse = sqrt( rmse/len );
      return rmse;
    }

    double find_slope_max(dataBuffer *x, dataBuffer *y) {
      int len = x->len();
      double slope_n;
      double slope_max = 0;
      for(int n=0; n<len; n++) {
        if( n != dataBufferPos ) {
          slope_n = (y->dataBufferData[dataBufferPos] - y->dataBufferData[n]) / (x->dataBufferData[dataBufferPos] - x->dataBufferData[n]);
          if(abs(slope_n) > slope_max)
            slope_max = abs(slope_n);
        }
      }
      return slope_max;
    }
};
