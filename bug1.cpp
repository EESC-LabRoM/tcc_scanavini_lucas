#include <obstacle_avoidance/bug1.h>
#include <obstacle_avoidance/math.h>

namespace bug{
    /**
    *  Empty constructor
    */
    Bug1::Bug1(){
        // Reserve memory space for 3 sonars.
        for(int i=0; i<2; i++){
            sonarArray.push_back(0);    
           
        }
    }
    
    /**
    *  Empty destructor
    */
    Bug1::~Bug1(){;}
    
    /**
    * Go to Point: Computes desired twist that takes the robot towards the goal point. 
    */
    void Bug1::goToPoint(void){
        // Go to point. Given the point specified in the this->goal, compute the necessary twist to reach it. HEre, you must use this->odometry.
                
        double teta;//angulo entre o robot and goal
        double dif;//diferença entre angulo do robo e teta desejado 
        double co;//cateto oposto
        double ca;//cateto adjacente
        double module;//modulo para controle quando muito proximo do goal
        geometry_msgs::Quaternion q = this->odometry.pose.pose.orientation;//armazena matriz em q
        double direc = tf::getYaw(q);//recebe o angulo do robo com a orizontal
        co=this->goal.y-this->odometry.pose.pose.position.y;
        ca=this->goal.x-this->odometry.pose.pose.position.x;
        teta= std::atan2(co,ca);
        dif=teta-direc;
        dif= math::normalizeAngle(dif);
        module = std::abs(dif);
        //caso diferença entre teta e angulo do robo muito pequena-virar sem velocidade linear
        if(0<=module<=0.0001){
             this->twist.linear.x=0.2;
            
        }
        //caso diferença seja significativa atue girando o robo
        else {
            this->twist.angular.z=dif;
            this->twist.linear.x=0.1;
           
        }
    }
    
    
    /**;
    * Wall Following function: Computes desired twist that allows robot circum-navagiating a wall/obstacle.
    */
    void Bug1::wallFollower(void){
        // Wall follower routine. Computes this->twist that enables the robot following an obstacle continuously. Try to use only local measurements, e.g. sonar sensors. No need to use odometry.           
        if((this->sonarArray[RIGHT_SONAR]<=0.5 && this->sonarArray[RIGHT_SONAR]>0)||(this->sonarArray[FRONT_SONAR]>0.1 && this->sonarArray[FRONT_SONAR]<=0.8)){
             this->twist.angular.z=0.2;
             this->twist.linear.x=0;
        }//vira para esquerda sempre que encontrar um obstaculo
        
        else if(this->sonarArray[RIGHT_SONAR]<=0.6 && this->sonarArray[RIGHT_SONAR]>0.5){
             this->twist.angular.z=0;
             this->twist.linear.x=0.2;
      
        }//siga a lateral do obstaculo
       
        else if((this->sonarArray[RIGHT_SONAR]<=0.8 && this->sonarArray[RIGHT_SONAR]>0.6 && this->sonarArray[FRONT_SONAR]==0)||(this->sonarArray[RIGHT_SONAR]==0 && this->sonarArray[LEFT_SONAR]==0)){
             this->twist.angular.z=-0.2;
             this->twist.linear.x=0.08;
  
        }//se afastando do obstaculo VOLTE JA!!!
            
   }
         

    
    /**
    * Bug Manager: Decides which sub-routine shall be called.
    */
    void Bug1::bugManager(void){
        // Static variables hold values
        static int state = 0; 
        static double shortestDistanceToGoal = 0;
        double counta=0;
        // Compute current distance in respect to the final goal 
        double distanceToGoal = math::distance2D(this->odometry.pose.pose.position, this->goal);
        // Compute current distance in respect to last hit point
        double distanceToH_in = math::distance2D(this->odometry.pose.pose.position, this->h_in);
        // Compute current distance in respect to last leaving point
        double distanceToH_out = math::distance2D(this->odometry.pose.pose.position, this->h_out);
       
        
        std::cout << "state: " << state << std::endl;
        std::cout << "distance to goal: " << distanceToGoal << std::endl;
        std::cout << "distance to hin: " << distanceToH_in << std::endl;
        std::cout << "distance to hout: " << distanceToH_out << std::endl;

        switch (state){
                 // State 0: Nothing to do.
            case 0: if (distanceToGoal > 0.1){ // Robot is far from the goal
                        // Change to "Go to point" state.
                        state = 1;
                    }
                    break;
                
            // State 1: Obstacle free, pursue the goal!
            case 1: // Move toward the goal.
                    this->goToPoint(); 
                    // Did the robot reach the goal?
                    if (distanceToGoal < 0.05){  
                        // Change to "resting" state.
                        this->twist.linear.x = 0;
                        this->twist.angular.z = 0;
                        state = 0;
                    } else
                    // Did the robot detected an obstacle in front of it? 
                    if (this->sonarArray[FRONT_SONAR] > 0 && this->sonarArray[FRONT_SONAR] < 0.5){
                        // Save hit IN point.
                        h_in = odometry.pose.pose.position;
                        // Change to "obstacle detected" state.
                        state = 2;
                    }
                    break;
            
            // State 2: The robot has just detected an obstacle.
            case 2: // Follow the wall.
                    this->wallFollower();
                    // Remain in this state until robot is far enough from hit point.
                    if (distanceToH_in  > 0.5){
                        // So far, this is shortest distance to goal.
                        shortestDistanceToGoal = distanceToGoal;
                        // Change "circum-navigate" state.
                        state = 3;
                    }
                    break;
            
            // State 3: The robot must circum-navigate the obstacle.
            case 3: // Follow the wall.
                    this->wallFollower();
                    // Is robot closer to the goal than ever before?
                    if(distanceToGoal < shortestDistanceToGoal){
                        // Yes! Then save current position.
                        this->h_out = this->odometry.pose.pose.position;
                        shortestDistanceToGoal = distanceToGoal;
                    }
                    // Remain in this state until robot is back to initial hit point (hit IN).
                    if (distanceToH_in < 0.4){
                        // Change "back to closest point" state.
                        state = 4;
                    }
                    break;
                
            // State 4: Take the robot back to the closest point in respect to final goal
            case 4: // Follow the wall.
                    this->wallFollower();
                    // Remain in this state until robot is back to leaving point (hit OUT).
                    if (distanceToH_out < 0.1){
                        // change state.
                        state = 0;
                    }
                    break;
                                        
        } 
        
    }
    
} // closing bug namespace







