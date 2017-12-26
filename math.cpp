#include <obstacle_avoidance/math.h>

namespace math{
/**
* Distance: Computes the 3D distance betweem points p1 and p2.
*/
double distance(geometry_msgs::Point &p1, geometry_msgs::Point &p2){
    return sqrt(pow(p1.x-p2.x,2) + pow(p1.y-p2.y,2) + pow(p1.z-p2.z,2) );
}

/**
* Distance 2D: Computes the 2D distance betweem points p1 and p2.
*/
double distance2D(geometry_msgs::Point &p1, geometry_msgs::Point &p2){
    return sqrt(pow(p1.x-p2.x,2) + pow(p1.y-p2.y,2) );
}
    
/**
* Normalize angle: Return the normalized angle from -PI to PI
*/
    
double normalizeAngle(double theta){
    if (theta > M_PI){
        theta = theta - 2*M_PI;
    } else if (theta < -M_PI){
        theta = theta + 2*M_PI;
    }
    return theta;
}
//Angle between goal and robot to find M-LINE angain
void getmline(geometry_msgs::Point &pr, geometry_msgs::Point &pg, double &mcoeficient, double &mb, int &flag){
   double cx = pg.x-pr.x;
   double cy = pg.y-pr.y;
   if (std::abs(cx)<=0.1){
       flag=0;
       mb=pr.x;
   }
   else if (std::abs(cy)<=0.1){
       flag=1;
       mb=pr.y;
   }
   else if(std::abs(cx)>0.1||std::abs(cy)>0.1) {
       flag=3;
       mcoeficient=cy/cx;
       mb=pr.y-mcoeficient*pr.x;
   }
}
//retorna 0 caso caminho livre, retorna 1 caso obstruido
 int isPathFree(geometry_msgs::Quaternion &q,geometry_msgs::Point &goal, geometry_msgs::Point &pr,  double LEFT,double RIGHT,double FRONT){
       double teta;//angulo entre o robot and goal
       double dif;//diferença entre angulo do robo e teta desejado 
       double co;//cateto oposto
       double ca;//cateto adjacente
       double direc = tf::getYaw(q);//recebe o angulo do robo com a orizontal
       co= goal.y-pr.y;
       ca= goal.x-pr.x;
       teta= std::atan2(co,ca);
       dif=teta-direc;
       dif= normalizeAngle(dif);
       std::cout << "DIF: " << dif << std::endl;
        if ((dif>0 && LEFT >0 && LEFT <0.8 )||(dif<0 &&  RIGHT >0 && RIGHT <0.8)||FRONT!=0){
            return 1;}
        else {
        return 0;}
}
}