// Simulation of adaptive DC machine control to be run on micro controller.

// This program simulates two parts.  The machine and the controller action.
// Controller action consists of both acquiring model of a motor under control,
// and then controlling the speed in an attempt to reach a required position.
// The machine model is acquired by finding the state equation characteristic
// parameters.
// The parameters are found by applying the least squares method on the
// current and speed samples which were taken.

// The application we were looking at was a DC machine coupled to a synchronous
// generator.  The AC machine acts as the speed and position feedback by just
// taking note of the zero crossings.  Real-world noise are ignored for now,
// but will bother you in a real implementation...
// The machine is meant to be controlled by adjusting the firing angle of an SCR,
// but PWM on IGBTs can also be done.

// The program spews forth a lot of tab-separated data, which you can pipe to a
// CSV file and plot using Gnuplot or excel.

// It also drops the model parameters on cerr.  Which you can load in your next // simulation by just passing them on as args on the command line.

// This is an idea by Jean Fivaz --- 2014/05/02

// Use responsibly...
 
#include <iostream>
#include <cmath>
#include <cstdlib>

#define FLOAT double
#define COUT std::cout <<
#define ENDL << std::endl
#define TAB << '\t' <<
#define EXP std::exp
#define SIN std::sin
#define COS std::cos
#define PI std::M_PI
#define ATOF std::atof
#define FABS std::fabs

#define POSITION_CONTROL

const FLOAT La = 5.5f;//0.5f;//5.5f;
const FLOAT kv = 0.022f;//0.01f;//0.022f;
const FLOAT J =  0.0011f;//0.00011f;
const FLOAT kt = 0.11f;//0.01f;//0.11f;
const FLOAT dt = 0.00002f;
const FLOAT B = 0.011f;//0.1f;//0.0011f;
const FLOAT TL = 1.0f;
const FLOAT Ra = 6.6f;//1.0f;//6.6f;

FLOAT watn = 0.0f;
FLOAT watn_1 = 0.0f;
FLOAT iatn = 0.0f;
FLOAT iatn_1 = 0.0f;
FLOAT vatn = 0.0f;
FLOAT vatn_1 = 0.0f;

FLOAT** inv (FLOAT** m, unsigned int size) {
  FLOAT** T = new FLOAT* [size];
  FLOAT** I = new FLOAT* [size];
  for (unsigned i = 0; i < size; i++) {
    T [i] = new FLOAT [size];
    I [i] = new FLOAT [size];
    for (unsigned j = 0; j < size; j++) {
      T [i][j] = m [i][j];
      I [i][j] = i == j ? 1.0f : 0.0f;
    }
  }
  for (unsigned refrow = 0; refrow < size; refrow++) {
    FLOAT refdiv = T [refrow][refrow];
    for (unsigned entry = 0; entry < size; entry++) {
      if (!(entry < refrow)) T [refrow][entry] /= refdiv;
      I [refrow][entry] /= refdiv;
    }
    for (unsigned row = refrow + 1; row < size; row++) {
      FLOAT rowdiv = T [row][refrow];
      for (unsigned entry = 0; entry < size; entry++) {
	if (!(entry < refrow)) T [row][entry] -= rowdiv * T [refrow][entry]; 
	I [row][entry] -= rowdiv * I [refrow][entry];
      }
    }
  }
  for (int refrow = size - 1; refrow > 0; refrow--) {
    FLOAT rowdiv = T [refrow - 1][refrow];
    for (int row = refrow - 1; row > -1; row--) {
      for (int entry = 0; entry < size; entry++) {
	if (!(entry < refrow)) T [row][entry] -= rowdiv * T [refrow][entry];
	I [row][entry] -= rowdiv * I [refrow][entry];
      }
    }
  }
  for (unsigned j = 0; j < size; j++) delete [] T [j];
  delete [] T;
  return I;
}

void print (FLOAT** m, unsigned size) {
  for (unsigned i = 0; i < size; i++) {
    for (unsigned j = 0; j < size; j++) {
      COUT m [i][j];
      COUT '\t';
    }
    COUT std::endl;
  }
  return;
}

void print (FLOAT* m, unsigned size) {
  for (unsigned j = 0; j < size; j++) {
    COUT m [j];
    COUT std::endl;
  }
  return;
}


FLOAT** multiply (FLOAT** a, FLOAT** b, unsigned size) {
  FLOAT** m = new FLOAT* [size];
  for (unsigned i = 0; i < size; i++) {
    m [i] = new FLOAT [size];
    for (unsigned j = 0; j < size; j++) {
      m [i][j] = 0.0f;
      for (unsigned k = 0; k < size; k++) {
	m [i][j] += a [i][k] * b [k][j];
      }
    }
  }
  return m;
}

FLOAT* multiply (FLOAT** a, FLOAT* b, unsigned size) {
  FLOAT* m = new FLOAT [size];
  for (unsigned j = 0; j < size; j++) {
    m [j] = 0.0f;
    for (unsigned k = 0; k < size; k++) {
      m [j] += a [j][k] * b [k];
    }
  }
  return m;
}

FLOAT** multiply (FLOAT a, FLOAT** b, unsigned size) {
  FLOAT** m = new FLOAT* [size];
  for (unsigned i = 0; i < size; i++) {
    m [i] = new FLOAT [size];
    for (unsigned j = 0; j < size; j++) {
      m [i][j] = a * b [i][j];
    }
  }
  return m;
}

FLOAT* multiply (FLOAT a, FLOAT* b, unsigned size) {
  FLOAT* m = new FLOAT [size];
  for (unsigned j = 0; j < size; j++) {
    m [j] = a * b [j];
  }
  return m;
}

FLOAT** plus (FLOAT** a, FLOAT** b, unsigned size) {
  FLOAT** m = new FLOAT* [size];
  for (unsigned i = 0; i < size; i++) {
    m [i] = new FLOAT [size];
    for (unsigned j = 0; j < size; j++) {
      m [i][j] = a [i][j] + b [i][j];
    }
  }
  return m;
}

FLOAT* plus (FLOAT* a, FLOAT* b, unsigned size) {
  FLOAT* m = new FLOAT [size];
  for (unsigned j = 0; j < size; j++) {
    m [j] = a [j] + b [j];
  }
  return m;
}

FLOAT** minus (FLOAT** a, FLOAT** b, unsigned size) {
  FLOAT** m = new FLOAT* [size];
  for (unsigned i = 0; i < size; i++) {
    m [i] = new FLOAT [size];
    for (unsigned j = 0; j < size; j++) {
      m [i][j] = a [i][j] - b [i][j];
    }
  }
  return m;
}

FLOAT** transpose (FLOAT** a, unsigned rows, unsigned cols) {
  FLOAT** T = new FLOAT* [cols];
  for (unsigned i = 0; i < cols; i++) {
    T [i] = new FLOAT [rows];
    for (unsigned j = 0; j < rows; j++) {
      T [i][j] = a [j][i];
    }
  }
  return T;
}

FLOAT** multiply (FLOAT** a, unsigned arows, unsigned acols, FLOAT** b, unsigned bcols) {
  FLOAT** p = new FLOAT* [arows];
  for (unsigned i = 0; i < arows; i++) {
    p [i] = new FLOAT [bcols];
    for (unsigned j = 0; j < bcols; j++) {
      p [i][j] = 0.0f;
      for (unsigned k = 0; k < acols; k++) {
	p [i][j] += a [i][k] * b [k][j];
      }
    }
  }
  return p;
}

FLOAT* multiply (FLOAT** a, unsigned arows, unsigned acols, FLOAT* b) {
  FLOAT* p = new FLOAT [arows];
  for (unsigned j = 0; j < arows; j++) {
    p [j] = 0.0f;
    for (unsigned k = 0; k < acols; k++) {
      p [j] += a [j][k] * b [k];
    }
  }
  return p;
}

int main (int argc, char** argv){

  // Setting up the machine characteristics required for simulation.
  FLOAT** A = new FLOAT* [2];
  A [0] = new FLOAT [2];
  A [1] = new FLOAT [2];
  A [0][0] =-B / J;
  A [0][1] = kt / J;
  A [1][0] = -kv / La;
  A [1][1] = -Ra / La;

  FLOAT b [2] = {0.0f, 1.0f / La};
  FLOAT c [2] = {1.0f / J, 0.0f};

  FLOAT** I = new FLOAT* [2];
  for (unsigned i = 0; i < 2; i++) {
    I [i] = new FLOAT [2];
    for (unsigned j = 0; j < 2; j++) {
      I [i][j] = i == j ? 1.0f : 0.0f;
    }
  }

  // Setting up the simulation matrices.
  FLOAT** halfdtA = multiply (dt / 2.0f, A, 2);
  FLOAT** M = multiply (inv (minus (I, halfdtA, 2), 2),
			plus (I, halfdtA, 2), 2);

  FLOAT* N = multiply (inv (minus (I, halfdtA, 2), 2), 
		       multiply (dt / 2.0f, b, 2) , 2);

  // FLOAT* O = multiply (inv (minus (I, halfdtA, 2), 2), 
  // 		       multiply (dt / 2.0f, c, 2) , 2);

  FLOAT* x_1 = new FLOAT [2];
  x_1 [0] = 0.0f;
  x_1 [1] = 0.0f;
  FLOAT* x = new FLOAT [2];
  x [0] = 0.0f;
  x [1] = 0.0f;

  FLOAT g_1 = 0.0f;
  FLOAT h_1 = 0.0f;
  unsigned sampleindex = 0;
  FLOAT sample = 0.0f;
  FLOAT scale = 1.0f;

  FLOAT periods = 0.0f;
  FLOAT* measurements = new FLOAT [3];
  FLOAT* measurements_1 = new FLOAT [3];
  for (unsigned i = 0; i < 3; i++) {
    measurements [i] = 0.0f;
    measurements_1 [i] = 0.0f;
  }
  FLOAT angle = 0.0f;
  FLOAT angle_1 = 0.0f;
  FLOAT t_1 = 0.0f;

  FLOAT** Km = new FLOAT* [2];
  Km [0] = new FLOAT [3];
  Km [1] = new FLOAT [3];

  FLOAT* y = new FLOAT [2];
  FLOAT u = 1.0f;
    
  FLOAT** XTY = new FLOAT* [2];
  XTY [0] = new FLOAT [3];
  XTY [0][0] = 0.0f;
  XTY [0][1] = 0.0f;
  XTY [0][2] = 0.0f;
  XTY [1] = new FLOAT [3];
  XTY [1][0] = 0.0f;
  XTY [1][1] = 0.0f;
  XTY [1][2] = 0.0f;

  FLOAT** XTX = new FLOAT* [3];
  for (unsigned i = 0; i < 3; i++) {
    XTX [i] = new FLOAT [3];
    for (unsigned j = 0; j < 3; j++) {
      XTX [i][j] = 0.0f;
    }
  }

  // Load previous experience acquired.    
  if (argc > 15) {
    unsigned k = 1; 
    for (unsigned i = 0; i < 3; i++) {
      for (unsigned j = 0; j < 3; j++) {
	XTX [i][j] = ATOF (argv [k++]);
      }
      XTY [0][i] = ATOF (argv [k++]);
      XTY [1][i] = ATOF (argv [k++]);
    }
  }

  // Current measurements are not taken as often as speed
  // measurements.
  FLOAT lastIreadingtime = 0.0f;
  FLOAT Iperiod = 0.005f;

  // Simulation starts here -----------------
  for (FLOAT t = 0.0f; t < 33.0f; t += dt) {
    FLOAT g = //180.0f * u; // 
      254.558f * SIN (M_PI * 100.0f * t) * u;
    g = (g >= 0.0f) ? g : -g;
    if (t < dt) g = 0.0f;
    
    FLOAT h = 0.0f;//(t > 0.0f ? 15.5f : 20.0f);
    //    if (sampleindex > 8000) h = -120000.0f;
    x = plus (multiply (M, x_1, 2),
	      //     plus (
	      multiply (g_1 + g, N, 2),
	      //	    multiply (h_1 + h, O, 2),
	      //    2),
	      2);

    angle += (x [0] + x_1 [0]) * dt / 2.0f;


    // Control starts here ---------------
    // Time between zero crossings are recorded.
    if ((SIN (angle) * SIN (angle_1) < 0.0f) || t < dt) { 
      if (t < dt) {
	sample = 0.0f;
	periods = 0.0f;
	measurements_1 [0] = measurements [0] = 0.0f;
	measurements_1 [1] = measurements [1] = 0.0f;
	sampleindex = 0;
	y [0] = 0.0f;
	y [1] = 0.0f;
      } else {
	sample = t;
	periods = t - t_1;
	measurements_1 [0] = measurements [0];
	measurements [0] = M_PI / periods;
	sampleindex++;
	
	// Control calculations start here:
	// With every current reading.
	// (Periods are sampled more finely than current.)
	if (t - lastIreadingtime > 0.01f ) { // simulate delay in taking reading
	  measurements_1 [1] = measurements [1];
	  measurements [1] = x [1];
	  Iperiod = t - lastIreadingtime;
	  lastIreadingtime = t;
	  measurements [2] = g; //180.0f * u;
	  
	  // Change in speed over time
	  y [0] = (measurements [0] - measurements_1 [0]) / periods;
	  
	  // Change in current over time
	  y [1] = (measurements [1] - measurements_1 [1]) / Iperiod;

	  // This is the actual "learning".
	  // History of measurements are  kept, but rather averaged using
	  // minimum squares method.
	  XTX [0][0] += measurements [0] * measurements [0];
	  XTX [0][1] += measurements [1] * measurements [0];
	  XTX [1][0] = XTX [0][1];
	  XTX [0][2] += measurements [2] * measurements [0];
	  XTX [2][0] = XTX [0][2];
	  XTX [1][1] += measurements [1] * measurements [1];
	  XTX [1][2] += measurements [2] * measurements [1];
	  XTX [2][1] = XTX [1][2];
	  XTX [2][2] += measurements [2] * measurements [2];
	  XTY [0][0] += y [0] * measurements [0];
	  XTY [0][1] += y [0] * measurements [1];
	  XTY [0][2] += y [0] * measurements [2];
	  XTY [1][0] += y [1] * measurements [0];
	  XTY [1][1] += y [1] * measurements [1];
	  XTY [1][2] += y [1] * measurements [2];
	  
	  //  T         T
	  // M .M.Km = M .y
	  
	  //        T   -1  T
	  // Km = (M .M)  .M .y
	  
	  // 
	}
	if (argc > 15 || sampleindex > 7) {

	  // The estimate for the motor's characteristics:
	  Km [0] = multiply (inv (XTX, 2), XTY [0], 2);
	  Km [1] = multiply (inv (XTX, 3), XTY [1], 3);

	  FLOAT pu = u;

	  FLOAT POSITION = 25.0f;

	  // Predict the speed the motor with Km characteristics is to
	  // converge to / calculate the voltage required to make the
	  // motor converge to the required speed.
	  u = ((t > 30.0f) ? 0.0f : -1256.0f
#ifdef POSITION_CONTROL	       
	       // speed profile required for positioning control
	       * POSITION / M_PI / Km [0][1]
	       * COS (M_PI * (0.5f * (FLOAT) sampleindex / POSITION))) 
	       //* (((0 < sampleindex) && (sampleindex < 9000)) ? 3.0f : 0.0f))
#endif // POSITION_CONTROL
	    * (Km [1][0] - Km [1][1] * Km [0][0] / Km [0][1]) 
	    / Km [1][2] 
	    // the reason or cause of this factor is unknown, but rest
	    // assure that this control doesn't work without it, or as
	    // well if the value changes.  Maybe one day I'll know.
	    / 100000.0f;

#ifdef SPEED_CONTROL
	  // Proportional control only for speed control.  Gets added
	  // to predictive control as required in degrees: depends on
	  // the change in u in time.
	  FLOAT dpu = FABS (u - pu) + 0.125f;
	  u = u * (dpu)
	    + (1.0f - (dpu)) * pu 
	    * periods
	    * ((t > 30.0f) ? 0.0f : 400.0f);
#endif // SPEED_CONTROL
	  // Range limiting.  To be multiplied with micro pwm factor. (255)
	  if (u > 1.0f) u = 1.0f;
	  if (u < 0.0f) u = 0.0f;
	}
	COUT t TAB sampleindex
	  TAB measurements [0] TAB u TAB Km [0][0]
	  TAB Km [0][1] TAB Km [1][0] TAB Km [1][1] TAB Km [1][2]
	  TAB x [1] ENDL;
      } // End of control calculations.

      t_1 = t;
    }      // End of control -----------------

    //    COUT t TAB x [0] TAB x [1] TAB g ENDL;
    angle_1 = angle;
    g_1 = g;
    h_1 = h;
    x_1 [0] = x [0];
    x_1 [1] = x [1];
  } // End of simulation -------------

  // Dump experience.  To be used and built on next time.
  for (unsigned i = 0; i < 3; i++) {
    for (unsigned j = 0; j < 3; j++) {
      std::cerr << XTX [i][j] << ' ';
    }
    std::cerr << XTY [0][i] << ' ';
    std::cerr << XTY [1][i] << ' ';
  }
  std::cerr ENDL;
  return 0;
}
