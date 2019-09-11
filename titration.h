// BD 10 mL Syringe Mechanical Specifications
//
// 10 mL, 14.5 mm ID, 1.6513 cm^2 Area, 1 mm Height = 0.16513 mL = 165.13 uL (165.13 uL per mm/3200 steps)
// 
// 1 step = 0.0516031 uL <-- this number is to be calibrated.
//
// Dynamic titrant dispense scheme (finite difference formula)
// Titration Scheme:
//   0) Initialize pH of the solution to pH ~3, wait for stable pH to begin
//   1) Dispense TITRATE_VOL (beginning with default TITRATE_VOL_NORM)
//   2) Measure/Calculate at every DELTA_T: pH_value, 
//                                          pH_1st_derivative (3 data points),
//                                          pH_2nd_derivative (3 data points),
//                                          pH_3rd_derivative (5 data points).
//      (Storing 5 time-series data points at any given time for the calculation)
//   3) After DELTA_T, if current pH_value is within pH_DELTA_STABLE from the last value
//     3) YES -> RECORD TIME, LOG DATA, set TITRATE_VOL, back to 1) to dispense again
//     3) NO -> back to 2), do not dispense
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
//        TITRATE_VOL (pH_1st_derivative) = TITRATE_VOL_MAX * EXP (- a * pH_1st_derivative) + TITRATE_VOL_MIN;
//   where a is a tuning variable. A larger a value means to prefer smaller steps (higher resolution) when the slope is high.
//   default a = 2;
// double TUNE_A = 2.0;
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

double pump_uL_PER_STEP = 0.0516031; // With syringe pump operating at 3200 steps/resolution = 1 mm, a 10 mL BD syringe gives
                                     // a theoretical resolution of <this number> (uL per microstep).
const int TITRATE_VOL_NORM = 25;    // default to 25 uL per step
const int TITRATE_VOL_MIN  = 5;     // default to 25 uL per step
const int TITRATE_VOL_MAX  = 100;   // default to 25 uL per step

const double pH_DELTA_STABLE = 0.03; // Accepted stable pH

const double DELTA_T = 5;           // default stablizing time interval (this can be dynamically tuned)

const int TITRATE_STEP_NORM = floor(TITRATE_VOL_NORM/pump_uL_PER_STEP);
const int TITRATE_STEP_MIN  = floor(TITRATE_VOL_MIN/pump_uL_PER_STEP);
const int TITRATE_STEP_MAX  = floor(TITRATE_VOL_MAX/pump_uL_PER_STEP);

const double TUNE_A = 2.0;

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
  
    int len() {
      return min(dataBufferFill,dataBufferLen);    
    }

    bool isFilled() {
      if (dataBufferFill >= dataBufferLen)
        return 1;
      else
        return 0;
    }

    double mean() {
      double sum = 0.;
      int len = this->len();
      if( len > 0 ) {
        for(int n=0;n<len;n++) {
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
      double data_value_tm1 = dataBufferData[(dataBufferPos-2+dataBufferLen) % dataBufferLen]; // -2
      double data_value_tp1 = dataBufferData[dataBufferPos];                                   // current

      return (data_value_tp1 - data_value_tm1)/(2.*t_delta);
    }

    double second_dev(double t_delta) { // 2nd_derivative
      double data_value_tm1 = dataBufferData[(dataBufferPos-2+dataBufferLen) % dataBufferLen]; // -2
      double data_value_t0  = dataBufferData[(dataBufferPos-1+dataBufferLen) % dataBufferLen]; // -1
      double data_value_tp1 = dataBufferData[dataBufferPos];                                   // current

      return (data_value_tp1 - data_value_t0 - data_value_t0 + data_value_tm1)/(t_delta*t_delta);
    }

    double third_dev(double t_delta) { // 3rd_derivative
      double data_value_tm2 = dataBufferData[(dataBufferPos-4+dataBufferLen) % dataBufferLen]; // -4
      double data_value_tm1 = dataBufferData[(dataBufferPos-3+dataBufferLen) % dataBufferLen]; // -3
      //double data_value_t0  = dataBufferData[(dataBufferPos-2+dataBufferLen) % dataBufferLen]; // -2
      double data_value_tp1 = dataBufferData[(dataBufferPos-1+dataBufferLen) % dataBufferLen]; // -1
      double data_value_tp2 = dataBufferData[dataBufferPos];                                   // current

      return (data_value_tp2 - 2.*data_value_tp1 + 2.*data_value_tm1 - data_value_tm2)/(2.*t_delta*t_delta*t_delta);
    }

    int compute_titration_Step_Volume (double t_delta) {
      int TITRATE_VOL = int ( double(TITRATE_VOL_MAX) * exp (-1.*double(TUNE_A)*this->first_dev(t_delta)) + double(TITRATE_VOL_MIN) );

      return TITRATE_VOL;
    }

    double compute_titration_uL_Volume (double t_delta) {
      return this->compute_titration_Step_Volume (t_delta) * pump_uL_PER_STEP;
    }

    double stdev() {
      double mean = this->mean();
      double sum = 0;
      int len = this->len();
      if( len > 0 ) {
        for(int n=0;n<len;n++) {
          sum+=pow(dataBufferData[n]-mean,2);
        }
        return sqrt(sum/len);
      } else
        return 0;
    }

    double linreg(dataBuffer *x,dataBuffer *y) {
      double mx=x->mean();
      double my=y->mean();

      double xmxdot=0.,xmxydot=0.,a,b,rmse;
      int len=x->len();
      for(int n=0;n<len;n++) {
        xmxdot+=pow(x->dataBufferData[n]-mx,2);
        xmxydot+=(x->dataBufferData[n]-mx)*y->dataBufferData[n];
      }
      a=xmxydot/xmxdot;
      b=my-a*mx;
      for(int n=0;n<len;n++) {
        rmse+=pow(y->dataBufferData[n]-a*x->dataBufferData[n]-b,2);
      }
      rmse=sqrt(rmse/len);
      return rmse;
    }
};
