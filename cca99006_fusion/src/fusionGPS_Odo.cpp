#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <cmath>
#include <Eigen/Dense> 
#include <tf2_ros/transform_broadcaster.h>

class Fusion: public rclcpp::Node{
    public:
    Fusion(void);
    private:
    void receptor_odom(const nav_msgs::msg::Odometry::SharedPtr odom_r); 
    void receptor_gps(const sensor_msgs::msg::NavSatFix::SharedPtr gps_r);
    void receptor_u(const geometry_msgs::msg::Twist::SharedPtr u_r); 
    void calcKalman(void);

    Eigen::Matrix3d P_kk_1; // P(k|k-1) AQUI
    Eigen::Vector3d x_kk_1; // x(k|k-1) AQUI

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_s; 
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_s; 

    //rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr fusionPosePublisher;
    rclcpp::TimerBase::SharedPtr timer_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_; //TF

    //nav_msgs::msg::Odometry odom;
    geometry_msgs::msg::Pose odom; 
    sensor_msgs::msg::NavSatFix gps;
    geometry_msgs::msg::Twist u;

    //nav_msgs::msg::Odometry fusedPose;
    geometry_msgs::msg::TransformStamped t;

};

void Fusion::receptor_odom(const nav_msgs::msg::Odometry::SharedPtr odom_r){
    //Pego a pose e twist do topico /odom do controlador
    odom = (*odom_r).pose.pose;
    u = (*odom_r).twist.twist;

}

void Fusion::receptor_gps(const sensor_msgs::msg::NavSatFix::SharedPtr gps_r){
    
    //recebe os dados do gps
    gps = *gps_r;
    //calcula o filtro 
    this->calcKalman();

    //publica o q foi calculado no topico /fused_pose
    //fusionPosePublisher->publish(fusedPose);

    //publica em /tf
    tf_broadcaster_->sendTransform(t);
}

Fusion::Fusion(void): Node("fusion_node"){
    using std::placeholders::_1;
    odom_s = create_subscription<nav_msgs::msg::Odometry>("/twist_mrac_linearizing_controller/odom",10,std::bind(&Fusion::receptor_odom,this,_1));
    
    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.best_effort();
    gps_s = create_subscription<sensor_msgs::msg::NavSatFix>("/gps/fix",qos,std::bind(&Fusion::receptor_gps,this,_1));
 
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    
    //fusionPosePublisher = create_publisher<nav_msgs::msg::Odometry>("/fused_pose",10);

    P_kk_1 << 5, 0, 0, //começo P(k|k-1) com valor qualquer aqui
              0, 5, 0, 
              0, 0, 5;
    
    //colocar chute inicial de x(k|k-1) tb
    x_kk_1 << 0,0,0; //começa na origem no gazebo

}

void Fusion::calcKalman(void){

    Eigen::MatrixXd K(3,2);
    Eigen::Matrix3d P_kk;  // P(k|k)
    Eigen::Matrix3d Pw;
    Eigen::MatrixXd H(2,3); //H
    Eigen::Matrix3d F;
    Eigen::Vector3d x_kk; // x(k|k)
    Eigen::Vector2d y; // y(k)
    Eigen::Vector2d h; // h( x(k|k-1),u(k) )

    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(3, 3);
    Eigen::MatrixXd B(3,2);
    Eigen::Vector2d U;

    Eigen::Matrix2d Pv;
    
    double T, lat_r, lon_r, theta_r;
    double f,Ex,Re,Rv,Rn,Res;
 
    //calculo das ctes
    // por ros2 topic hz /gps/fix  o average rate ta +- 4.3 Hz -> T = 0.23
    T = 0.23;
    lat_r = -30.027777778;
    lon_r = -51.228611111;
    theta_r = 0;

    f = 1/298.257223563;
    Ex = 2*f - pow(f,2);
    Re = 6378137.0;
    Rv = Re * sqrt(1/(1 - Ex*pow(sin(lat_r),2) ) );
    Rn = Rv*(1-Ex)*1/(1 - Ex*pow(sin(lat_r),2) );
    Res = Rv*cos(lat_r); 

    Pw <<   0.025, 0, 0,
            0, 0.025, 0,
            0,  0, 0.025;

    
    //Calculo do filtro em si
    Pv << gps.position_covariance[0], gps.position_covariance[1],
          gps.position_covariance[3], gps.position_covariance[4];
    
    y << gps.latitude, gps.longitude;


    //calculo H
    H <<  cos(theta_r)/Rn, sin(theta_r)/Rn, 0,
          cos(theta_r)/Res, sin(theta_r)/Res, 0;
    
    //h( x(k|k-1),u(k) )
    h << lat_r + (x_kk_1[0]*cos(theta_r)+x_kk_1[1]*sin(theta_r))/Rn ,
         lon_r - (-x_kk_1[0]*cos(theta_r)+x_kk_1[1]*sin(theta_r))/Res;
    
    // K
    K = P_kk_1 * H.transpose() * (H*P_kk_1*(H.transpose()) + Pv).inverse();

    // x(k|k) 
    x_kk = x_kk_1 + K * (y - h);
    // P(k|k)
    P_kk = (I - K*H)*P_kk_1;


    B << T*cos(x_kk[2]), 0,
         T*sin(x_kk[2]), 0,
         0             , T;

    Eigen::MatrixXd T_twilOdom(4,4);
    double theta_t = 2*atan2(odom.orientation.z,odom.orientation.w);
    double x_t = odom.position.x;
    double y_t = odom.position.y;

    T_twilOdom << cos(theta_t),-sin(theta_t),0,x_t,
                  sin(theta_t), cos(theta_t),0,y_t,
                  0           , 0           ,1,0,
                  0           , 0           ,0,1;
    
    Eigen::Matrix3d R_twilOdom;
    theta_r = 2*atan2(odom.orientation.z,odom.orientation.w);
    //Poderia fazer um slicing de T_twilOdom
    R_twilOdom << cos(theta_r),-sin(theta_r),0,
                  sin(theta_r), cos(theta_r),0,
                  0           , 0           ,1;
    
    Eigen::Vector3d uu;
    uu << u.linear.x, u.linear.y, u.angular.z;
    //Passo u (vel lin x e y + vel ang z) de odom -> twil e coloco em U
    U << ((R_twilOdom.transpose())*uu)(0), u.angular.z;
    // S = So + v*t classico
    x_kk_1 = x_kk + B*U; //faço a previsão x(k+1|k) que vai ser o x(k|k-1) da prox 

    F <<  1,0,-T*U[0]*sin(x_kk[2]),
          0,1, T*U[0]*cos(x_kk[2]),
          0,0,1;
    
    P_kk_1 = F*P_kk*(F.transpose()) + Pw; //P(k+1|k) que vai ser o P(k|k-1) da prox 

                  
    Eigen::MatrixXd T_twilMap(4,4);
    T_twilMap <<  cos(x_kk[2]),-sin(x_kk[2]),0,x_kk[0],
                  sin(x_kk[2]), cos(x_kk[2]),0,x_kk[1],
                  0           , 0           ,1,0,
                  0           , 0           ,0,1;
    
    //montagem da publicação do resultado da fusão de dados para TF
	//tenho twil->map (fusão de dados)
	//tenho twil->odom 
	//quero odom->map : achar o caminho das relações

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "map";
    t.child_frame_id = "odom";

    Eigen::MatrixXd T_OdomMap(4,4); //T de odom para map

    T_OdomMap = T_twilOdom * T_twilMap.inverse();
    
    t.transform.translation.x = T_OdomMap(3);
    t.transform.translation.y = T_OdomMap(7);
    t.transform.translation.z = 0.0;
    //na matriz de Transformação para o sis 2D em xy sempre aparece um cos e sin nas posições 0,0 e 1,0 assim que eu peguei o angulo(theta_t) 
    theta_t = atan2(T_OdomMap(4),T_OdomMap(0));

    t.transform.rotation.x = 0;
    t.transform.rotation.y = 0;
    t.transform.rotation.z = sin(theta_t/2); 
    t.transform.rotation.w = cos(theta_t/2);
    
    RCLCPP_INFO_STREAM(get_logger(),"P(k|k-1): \n "<<P_kk_1);
    RCLCPP_INFO_STREAM(get_logger(),"\n--------------\n");
    RCLCPP_INFO_STREAM(get_logger(),"vel em twil : \n "<<((R_twilOdom.transpose())*uu));
    RCLCPP_INFO_STREAM(get_logger(),"\n--------------\n");
    

}


int main(int argc, char ** argv)
{
  rclcpp::init(argc,argv);

  rclcpp::spin(std::make_shared<Fusion>());

  rclcpp::shutdown();

  return 0;
}



/*
    //montagem da publicação do resultado da fusão de dados no tópico fused_pose
    //Estava antes da montagem da publicacao em tf
    rclcpp::Time now = this->get_clock()->now();
    fusedPose.header.stamp = now;
    fusedPose.header.frame_id = "map"; 
    fusedPose.child_frame_id = "twil_origin";
    
    fusedPose.pose.pose.position.x = x_kk[0]; //map
    fusedPose.pose.pose.position.y = x_kk[1];
    fusedPose.pose.pose.position.z = 0.0;

    fusedPose.pose.pose.orientation.x = 0; //0
    fusedPose.pose.pose.orientation.y = 0; //0
    fusedPose.pose.pose.orientation.z = sin(x_kk[2]/2); //sin (o/2)
    fusedPose.pose.pose.orientation.w = cos(x_kk[2]/2); //cos (o/2)

    Eigen::Vector4d Temp;
    Temp << U[0],0,U[1],1;
    Temp = T_twilMap * Temp;

    fusedPose.twist.twist.linear.x = Temp[0];
    fusedPose.twist.twist.linear.y = Temp[1];
    fusedPose.twist.twist.angular.z = Temp[2];
    
    for(int i=0;i<3;i++)
      for(int j=0;j<3;j++){
           fusedPose.pose.covariance[j+6*i] = P_kk(i,j); //convariance é um vetor, por isso fiz assim
           fusedPose.twist.covariance[j+6*i] = Pw(i,j);
        }
*/