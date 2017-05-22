//****************************************************
//* quaternion.c++                                   *
//*                                                  *
//* Implementaion for a generalized quaternion class *   
//*                                                  *
//* Written 1.25.00 by Angela Bennett                *
//****************************************************

#include "quaternion.hpp"
#include "opencv2/imgproc/imgproc.hpp"

//Quaternion
// -default constructor
// -creates a new quaternion with all parts equal to zero
Quaternion::Quaternion(void)
{
  x = 0;
  y = 0;
  z = 0;
  w = 0;
}

Quaternion::Quaternion(cv::Mat rotationMatrix) {

  float r11 = rotationMatrix.at<float>(0,0);
  float r12 = rotationMatrix.at<float>(0,1);
  float r13 = rotationMatrix.at<float>(0,2);
  float r21 = rotationMatrix.at<float>(1,0);
  float r22 = rotationMatrix.at<float>(1,1);
  float r23 = rotationMatrix.at<float>(1,2);
  float r31 = rotationMatrix.at<float>(2,0);
  float r32 = rotationMatrix.at<float>(2,1);
  float r33 = rotationMatrix.at<float>(2,2);

  w = ( r11 + r22 + r33 + 1.0f) / 4.0f;
  x = ( r11 - r22 - r33 + 1.0f) / 4.0f;
  y = (-r11 + r22 - r33 + 1.0f) / 4.0f;
  z = (-r11 - r22 + r33 + 1.0f) / 4.0f;

  if(w < 0.0f) w = 0.0f;
  if(x < 0.0f) x = 0.0f;
  if(y < 0.0f) y = 0.0f;
  if(z < 0.0f) z = 0.0f;
  w = sqrt(w);
  x = sqrt(x);
  y = sqrt(y);
  z = sqrt(z);

  if(w >= x && w >= y && w >= z) {
      w *= +1.0f;
      x *= SIGN(r32 - r23);
      y *= SIGN(r13 - r31);
      z *= SIGN(r21 - r12);
  } else if(x >= w && x >= y && x >= z) {
      w *= SIGN(r32 - r23);
      x *= +1.0f;
      y *= SIGN(r21 + r12);
      z *= SIGN(r13 + r31);
  } else if(y >= w && y >= x && y >= z) {
      w *= SIGN(r13 - r31);
      x *= SIGN(r21 + r12);
      y *= +1.0f;
      z *= SIGN(r32 + r23);
  } else if(z >= w && z >= x && z >= y) {
      w *= SIGN(r21 - r12);
      x *= SIGN(r31 + r13);
      y *= SIGN(r32 + r23);
      z *= +1.0f;
  } else {
      printf("coding error\n");
  }

  float r = NORM(w, x, y, z);
  w /= r;
  x /= r;
  y /= r;
  z /= r;

}


//Quaternion
// -constructor
// -parametes : x, y, z, w elements of the quaternion
// -creates a new quaternion based on the elements passed in
Quaternion::Quaternion(float wi, float xi, float yi, float zi)
{
  w = wi;
  x = xi;
  y = yi;
  z = zi;
}

cv::Mat Quaternion::toRotationMatrix() {
	double xx = x * x, xy = x* y, xz = x * z, xw = x* w;
	double yy = y * y, yz = y * z, yw = y * w;
	double zz = z * z, zw = z * w;

	cv::Mat rotationMatrix(3,3, CV_32FC1);

	rotationMatrix.at<float>(0, 0) = (1 - 2 * yy - 2 * zz);
	rotationMatrix.at<float>(0, 1) = (2 * xy - 2 * zw);
	rotationMatrix.at<float>(0, 2) = (2 * xz + 2 * yw);

	rotationMatrix.at<float>(1, 0) = (2 * xy + 2 * zw);
	rotationMatrix.at<float>(1, 1) = (1 - 2 * xx - 2 * zz);
	rotationMatrix.at<float>(1, 2) = (2 * yz - 2 * xw);

	rotationMatrix.at<float>(2, 0) = (2 * xz - 2 * yw);
	rotationMatrix.at<float>(2, 1) = (2 * yz + 2 * xw);
	rotationMatrix.at<float>(2, 2) = (1 - 2 * xx - 2 * yy);

	return rotationMatrix;
}
 

//Quaternion
// -constructor
// -parameters : vector/array of four elements
// -creates a new quaternion based on the elements passed in
Quaternion::Quaternion(float v[4])
{
  w = v[0];
  x = v[1];
  y = v[2];
  z = v[3];
}


//Quaternion
// -copy constructor
// -parameters : const quaternion q
// -creates a new quaternion based on the quaternion passed in
Quaternion::Quaternion(const Quaternion& q)
{
  w = q.w;
  x = q.x;
  y = q.y;
  z = q.z;
} 

#ifdef SHOEMAKE
//Quaternion
// -constructor
// -parameters : yaw, pitch, and roll of an Euler angle
// -creates a new quaternion based on the Euler elements passed in
// -used with Shoemakes code
Quaternion::Quaternion(float e[3], int order)
{
  EulerAngles ea;
  ea.x = e[0];
  ea.y = e[1];
  ea.z = e[2];
  ea.w = order;

  Quat q = Eul_ToQuat(ea);

  x = q.x;
  y = q.y; 
  z = q.z;
  w = q.w;
}
#endif

//~Quaternion
// -destructor
// -deleted dynamically allocated memory
Quaternion::~Quaternion()
{
}


//operator=
// -parameters : q1 - Quaternion object
// -return value : Quaternion
// -when called on quaternion q2 sets q2 to be an object of  q3 
Quaternion Quaternion::operator = (const Quaternion& q)
{
  w = q.w;
  x = q.x;
  y = q.y;
  z = q.z;
  
  return (*this);
}

//operator+
// -parameters : q1 - Quaternion object
// -return value : Quaternion 
// -when called on quaternion q2 adds q1 + q2 and returns the sum in a new quaternion
Quaternion Quaternion::operator + (const Quaternion& q)
{
  return Quaternion(w+q.w, x+q.x, y+q.y, z+q.z);
}
  
//operator-
// -parameters : q1- Quaternion object
// -return values : Quaternion 
// -when called on q1 subtracts q1 - q2 and returns the difference as a new quaternion
Quaternion Quaternion::operator - (const Quaternion& q)
{
  return Quaternion(w-q.w, x-q.x, y-q.y, z-q.z);
}


//operator*
// -parameters : q1 - Quaternion object
// -return values : Quaternion 
// -when called on a quaternion q2, multiplies q2 *q1  and returns the product in a new quaternion 
Quaternion Quaternion::operator * (const Quaternion& q)
{
  return Quaternion(
   w*q.w - x*q.x - y*q.y - z*q.z, 
   w*q.x + x*q.w + y*q.z - z*q.y,                          
   w*q.y + y*q.w + z*q.x - x*q.z,
   w*q.z + z*q.w + x*q.y - y*q.x);
}
 
//operator/
// -parameters : q1 and q2- Quaternion objects
// -return values : Quaternion 
// -divide q1 by q2 and returns the quotient q1
Quaternion Quaternion::operator / (Quaternion& q)
{
  return ((*this) * (q.inverse()));
}


//operator+=
// -parameters : q1- Quaternion object
// -return values : Quaternion 
// -when called on quaternion q3, adds q1 and q3 and returns the sum as q3
Quaternion& Quaternion::operator += (const Quaternion& q)
{
  w += q.w;
  x += q.x;
  y += q.y;
  z += q.z;

  return (*this);
}


//operator-=
// -parameters : q1- Quaternion object
// -return values : Quaternion 
// -when called on quaternion q3, subtracts q1 from q3 and returns the difference as q3

Quaternion& Quaternion::operator -= (const Quaternion& q)
{
  w -= q.w;
  x -= q.x;
  y -= q.y;
  z -= q.z;

  return (*this);
}


//operator*=
// -parameters : q1- Quaternion object
// -return values : Quaternion 
// -when called on quaternion q3, multiplies q3 by q1 and returns the product as q3
 
Quaternion& Quaternion::operator *= (const Quaternion& q)
{
   float w_val = w*q.w - x*q.x - y*q.y - z*q.z;
   float x_val = w*q.x + x*q.w + y*q.z - z*q.y; 
   float y_val = w*q.y + y*q.w + z*q.x - x*q.z;
   float z_val = w*q.z + z*q.w + x*q.y - y*q.x; 
  
   w = w_val;
   x = x_val;
   y = y_val;
   z = z_val;

   return (*this);
}


//operator/=
// -parameters : q1- Quaternion object
// -return values : quaternion
// -when called on quaternion q3, divides q3 by q1 and returns the quotient as q3
 
Quaternion& Quaternion::operator /= (Quaternion& q)
{
  (*this) = (*this)*q.inverse();
  return (*this);
}


//operator!=
// -parameters : q1 and q2- Quaternion objects
// -return value : bool
// -determines if q1 and q2 are not equal

bool Quaternion::operator != (const Quaternion& q)
{
  return (w!=q.w || x!=q.x || y!=q.y || z!=q.z) ? true : false;
}

//operator==
// -parameters : q1 and q2- Quaternion objects
// -return value : bool
// -determines if q1 and q2 are equal

bool Quaternion::operator == (const Quaternion& q)
{
  return (w==q.w && x==q.x && y==q.y && z==q.z) ? true : false;
}  

//norm
// -parameters : none
// -return value : float
// -when called on a quaternion object q, returns the norm of q

float Quaternion::norm()
{
  return (w*w + x*x + y*y + z*z);  
}

//magnitude
// -parameters : none
// -return value : float
// -when called on a quaternion object q, returns the magnitude q

float Quaternion::magnitude()
{
  return sqrt(norm());
}

//scale
// -parameters :  s- a value to scale q1 by
// -return value: quaternion
// -returns the original quaternion with each part, w,x,y,z, multiplied by some scalar s

Quaternion Quaternion::scale(float  s)
{
   return Quaternion(w*s, x*s, y*s, z*s);
}

// -parameters : none
// -return value : quaternion
// -when called on a quaternion object q, returns the inverse of q

Quaternion Quaternion::inverse()
{
  return conjugate().scale(1/norm());
}

//conjugate
// -parameters : none
// -return value : quaternion
// -when called on a quaternion object q, returns the conjugate of q

Quaternion Quaternion::conjugate()
{
  return Quaternion(w, -x, -y, -z);
}
  
//UnitQuaternion
// -parameters : none
// -return value : quaternion
// -when called on quaterion q, takes q and returns the unit quaternion of q

Quaternion Quaternion::UnitQuaternion()
{
  return (*this).scale(1/(*this).magnitude());
}

// -parameters : vector of type float
// -return value : void
// -when given a 3D vector, v, rotates v by this quaternion

void Quaternion::QuatRotation(float v[3])
{
  Quaternion  qv(0, v[0], v[1], v[2]);
  Quaternion  qm = (*this) * qv * (*this).inverse();
  
  v[0] = qm.x;
  v[1] = qm.y;
  v[2] = qm.z;  
}

#ifdef SHOEMAKE
// -parameters : integer order- which will specify the order of the rotation, q- quaternion
// -return value : Euler angle
// -

void Quaternion::toEuler(float e[3], int order)
{
  Quat q;

  q.w = 0;
  q.x = e[0];
  q.y = e[1];
  q.z = e[2];

  EulerAngles ea = Eul_FromQuat(q, order);

  w = ea.w;
  x = ea.x;
  y = ea.y;
  z = ea.z;
}
#endif 
