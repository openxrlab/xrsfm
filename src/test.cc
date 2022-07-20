
#include <Eigen/Eigen> 
#include <iostream> 

double  frandom(){
  double  r = static_cast <double> (rand()) / static_cast <double> (RAND_MAX);
  return r;
}


int main(){
  using namespace Eigen;
  Vector3d p = Vector3d::Random();
  std::vector<double> d_vec(3);
  std::vector<Vector3d> v_vec(3);
  std::vector<Vector3d> t_vec(3);
  for(auto&v:v_vec){
    v = Vector3d::Random();
    v = v.normalized();
  }
  for(auto&d:d_vec){
    d = frandom();
  }
  for(int i = 0;i<3;++i){
    t_vec[i] = p-d_vec[i]*v_vec[i];
  }
  Matrix3d P,V,T,D;
  P<<p,p,p;
  V<<v_vec[0],v_vec[1],v_vec[2];
  T<<t_vec[0],t_vec[1],t_vec[2];
  D<<d_vec[0],0,0,0,d_vec[1],0,0,0,d_vec[2];
  std::cout<<P<<"\n V:"<<V<<"\n T:"<<T<<"\n D:"<<D<<std::endl; 

  Matrix3d _P = T+V*D;
  // std::cout<<V.inverse()*T+D<<std::endl;
  std::cout<<V.inverse()*T<<std::endl;

  // std::cout<<_P<<std::endl;
}