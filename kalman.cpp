
class Kalman {

private:
  float Q,R,F,H;
  float Xk,Pk;
  
public:
  
  Kalman();
  void Setup(float _q, float _r, float _f, float _h);
  float Correct(float Yk);
  void Reset(float _xk);
  
};

Kalman::Kalman() {
  Q = 1;
  R = 1;
  F = 1;
  H = 1;
  Xk = 0;
  Pk = 0;
}


void Kalman::Setup(float _q, float _r, float _f, float _h) {
  Q = _q;
  R = _r;
  F = _f;
  H = _h;
  Xk = 0;
  Pk = 0;
}


float Kalman::Correct(float Yk) {
  // Prediction
  float X0 = F * Xk;
  float P0 = F * Pk * F + Q;
  // Correction
  float K = P0 * H / (H * P0 * H + R);
  Xk = X0 + K * (Yk - H * X0);
  Pk = (1-K*H)*P0;
  return Xk;
}


void Kalman::Reset(float _xk) {
  Xk = _xk;
  Pk = 0;
}
