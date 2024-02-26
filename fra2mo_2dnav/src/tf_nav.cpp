#include "../include/tf_nav.h"
#include <tf/transform_broadcaster.h>



//4.b)////
std::vector<double> aruco_pose(7,0.0);

void aruco_call(const geometry_msgs::PoseStamped & msg)
{
    aruco_pose.clear();
    aruco_pose.push_back(msg.pose.position.x);
    aruco_pose.push_back(msg.pose.position.y);
    aruco_pose.push_back(msg.pose.position.z);
    aruco_pose.push_back(msg.pose.orientation.x);
    aruco_pose.push_back(msg.pose.orientation.y);
    aruco_pose.push_back(msg.pose.orientation.z);
    aruco_pose.push_back(msg.pose.orientation.w);

}

//4.c)//// Publish Aruco Pose as TF
void poseCallback(const geometry_msgs::PoseStamped & msg)
{
     static tf::TransformBroadcaster br;
     tf::Transform transform;
     transform.setOrigin( tf::Vector3(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z));
     tf::Quaternion q;
     q.setX(msg.pose.orientation.x);
     q.setY(msg.pose.orientation.y);
     q.setZ(msg.pose.orientation.z);
     q.setW(msg.pose.orientation.w);
     transform.setRotation(q);
     br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "aruco_frame"));

     }



TF_NAV::TF_NAV() {

    _position_pub = _nh.advertise<geometry_msgs::PoseStamped>( "/fra2mo/pose", 1 );
    _cur_pos << 0.0, 0.0, 0.0;
    _cur_or << 0.0, 0.0, 0.0, 1.0;

    _goal1_pos << 0.0, 0.0, 0.0;
    _goal1_or << 0.0, 0.0, 0.0, 1.0;

    _goal2_pos << 0.0, 0.0, 0.0;
    _goal2_or << 0.0, 0.0, 0.0, 1.0;

    _goal3_pos << 0.0, 0.0, 0.0;
    _goal3_or << 0.0, 0.0, 0.0, 1.0;

    _goal4_pos << 0.0, 0.0, 0.0;
    _goal4_or << 0.0, 0.0, 0.0, 1.0;

    _goal5_pos << 0.0, 0.0, 0.0;
    _goal5_or << 0.0, 0.0, 0.0, 1.0;

    _goal6_pos << 0.0, 0.0, 0.0;
    _goal6_or << 0.0, 0.0, 0.0, 1.0;

    _goal7_pos << 0.0, 0.0, 0.0;
    _goal7_or << 0.0, 0.0, 0.0, 1.0;

    _goal8_pos << 0.0, 0.0, 0.0;
    _goal8_or << 0.0, 0.0, 0.0, 1.0;

    _home_pos << -18.0, 2.0, 0.0;
}

void TF_NAV::tf_listener_fun() {
    ros::Rate r( 5 );
    tf::TransformListener listener;
    tf::StampedTransform transform;
    
    while ( ros::ok() )
    {
        try {
            listener.waitForTransform( "map", "base_footprint", ros::Time(0), ros::Duration(10.0) );
            listener.lookupTransform( "map", "base_footprint", ros::Time(0), transform ); //i take everything and i put in transform

        }
        catch( tf::TransformException &ex ) {
            ROS_ERROR("%s", ex.what());
            r.sleep();
            continue;
        }
        
        _cur_pos << transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z();
        _cur_or << transform.getRotation().w(),  transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z();
        position_pub();
        r.sleep();
    }

}


void TF_NAV::position_pub() {

    geometry_msgs::PoseStamped pose;

    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "map";



    pose.pose.position.x = _cur_pos[0];
    pose.pose.position.y = _cur_pos[1];
    pose.pose.position.z = _cur_pos[2];

    pose.pose.orientation.w = _cur_or[0];
    pose.pose.orientation.x = _cur_or[1];
    pose.pose.orientation.y = _cur_or[2];
    pose.pose.orientation.z = _cur_or[3];

    _position_pub.publish(pose);

}


//2.b) TF listeners ////////////////////////


void TF_NAV::goal_listener_1() {
    ros::Rate r( 1 );
    tf::TransformListener listener;
    tf::StampedTransform transform;

    while ( ros::ok() )
    {
        try
        {
            listener.waitForTransform( "map", "goal1", ros::Time( 0 ), ros::Duration( 10.0 ) );
            listener.lookupTransform( "map", "goal1", ros::Time( 0 ), transform );
        }
        catch( tf::TransformException &ex )
        {
            ROS_ERROR("%s", ex.what());
            r.sleep();
            continue;
        }

        _goal1_pos << transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z();
        _goal1_or << transform.getRotation().w(),  transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z();

        //ROS_INFO("Goal1 Position: %f %f %f", _goal1_pos[0], _goal1_pos[1], _goal1_pos[2]);
        //ROS_INFO("Goal1 Orientation: %f %f %f %f", _goal1_or[0], _goal1_or[1], _goal1_or[2], _goal1_or[3]);

        r.sleep();
    }    
}

/////////////////////////////////////////////////////////////////////////////////77


void TF_NAV::goal_listener_2() {
    ros::Rate r( 1 );
    tf::TransformListener listener;
    tf::StampedTransform transform;

    while ( ros::ok() )
    {
        try
        {
            listener.waitForTransform( "map", "goal2", ros::Time( 0 ), ros::Duration( 10.0 ) );
            listener.lookupTransform( "map", "goal2", ros::Time( 0 ), transform );
        }
        catch( tf::TransformException &ex )
        {
            ROS_ERROR("%s", ex.what());
            r.sleep();
            continue;
        }

        _goal2_pos << transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z();
        _goal2_or << transform.getRotation().w(),  transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z();

        //ROS_INFO("Goal2 Position: %f %f %f", _goal2_pos[0], _goal2_pos[1], _goal2_pos[2]);
        //ROS_INFO("Goal2 Orientation: %f %f %f %f", _goal2_or[0], _goal2_or[1], _goal2_or[2], _goal2_or[3]);

        r.sleep();
    }    
}


/////////////////////////////////////////////////////////////////////////////////

void TF_NAV::goal_listener_3() {
    ros::Rate r( 1 );
    tf::TransformListener listener;
    tf::StampedTransform transform;

    while ( ros::ok() )
    {
        try
        {
            listener.waitForTransform( "map", "goal3", ros::Time( 0 ), ros::Duration( 10.0 ) );
            listener.lookupTransform( "map", "goal3", ros::Time( 0 ), transform );
        }
        catch( tf::TransformException &ex )
        {
            ROS_ERROR("%s", ex.what());
            r.sleep();
            continue;
        }

        _goal3_pos << transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z();
        _goal3_or << transform.getRotation().w(),  transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z();

        //ROS_INFO("Goal3 Position: %f %f %f", _goal3_pos[0], _goal3_pos[1], _goal3_pos[2]);
        //ROS_INFO("Goal3 Orientation: %f %f %f %f", _goal3_or[0], _goal3_or[1], _goal3_or[2], _goal3_or[3]);

        r.sleep();
    }    
}

/////////////////////////////////////////////////////////////////////////////////

void TF_NAV::goal_listener_4() {
    ros::Rate r( 1 );
    tf::TransformListener listener;
    tf::StampedTransform transform;

    while ( ros::ok() )
    {
        try
        {
            listener.waitForTransform( "map", "goal4", ros::Time( 0 ), ros::Duration( 10.0 ) );
            listener.lookupTransform( "map", "goal4", ros::Time( 0 ), transform );
        }
        catch( tf::TransformException &ex )
        {
            ROS_ERROR("%s", ex.what());
            r.sleep();
            continue;
        }

        _goal4_pos << transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z();
        _goal4_or << transform.getRotation().w(),  transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z();

        //ROS_INFO("Goal4 Position: %f %f %f", _goal4_pos[0], _goal4_pos[1], _goal4_pos[2]);
        //ROS_INFO("Goal4 Orientation: %f %f %f %f", _goal4_or[0], _goal4_or[1], _goal4_or[2], _goal4_or[3]);
        r.sleep();
    }    
}

///////////////////////////////////////////////////////////////

//3.a) TF listeners to get a complete map of the environment
void TF_NAV::goal_listener_5() {
    ros::Rate r( 1 );
    tf::TransformListener listener;
    tf::StampedTransform transform;

    while ( ros::ok() )
    {
        try
        {
            listener.waitForTransform( "map", "goal5", ros::Time( 0 ), ros::Duration( 10.0 ) );
            listener.lookupTransform( "map", "goal5", ros::Time( 0 ), transform );
        }
        catch( tf::TransformException &ex )
        {
            ROS_ERROR("%s", ex.what());
            r.sleep();
            continue;
        }

        _goal5_pos << transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z();
        _goal5_or << transform.getRotation().w(),  transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z();
        r.sleep();
    }    
}

/////////////////////////////////////////////////////////////////////////////////
void TF_NAV::goal_listener_6() {
    ros::Rate r( 1 );
    tf::TransformListener listener;
    tf::StampedTransform transform;

    while ( ros::ok() )
    {
        try
        {
            listener.waitForTransform( "map", "goal6", ros::Time( 0 ), ros::Duration( 10.0 ) );
            listener.lookupTransform( "map", "goal6", ros::Time( 0 ), transform );
        }
        catch( tf::TransformException &ex )
        {
            ROS_ERROR("%s", ex.what());
            r.sleep();
            continue;
        }

        _goal6_pos << transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z();
        _goal6_or << transform.getRotation().w(),  transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z();
        r.sleep();
    }    
}

/////////////////////////////////////////////////////////////////////////////////
void TF_NAV::goal_listener_7() {
    ros::Rate r( 1 );
    tf::TransformListener listener;
    tf::StampedTransform transform;

    while ( ros::ok() )
    {
        try
        {
            listener.waitForTransform( "map", "goal7", ros::Time( 0 ), ros::Duration( 10.0 ) );
            listener.lookupTransform( "map", "goal7", ros::Time( 0 ), transform );
        }
        catch( tf::TransformException &ex )
        {
            ROS_ERROR("%s", ex.what());
            r.sleep();
            continue;
        }

        _goal7_pos << transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z();
        _goal7_or << transform.getRotation().w(),  transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z();
        r.sleep();
    }    
}


/////////////////////////////////////////////////////////////////////////////////
////3.b) TF listener useful to send the robot in the proximity of obstacle 9
void TF_NAV::goal_listener_8() {
    ros::Rate r( 1 );
    tf::TransformListener listener;
    tf::StampedTransform transform;

    while ( ros::ok() )
    {
        try
        {
            listener.waitForTransform( "map", "goal8", ros::Time( 0 ), ros::Duration( 10.0 ) );
            listener.lookupTransform( "map", "goal8", ros::Time( 0 ), transform );
        }
        catch( tf::TransformException &ex )
        {
            ROS_ERROR("%s", ex.what());
            r.sleep();
            continue;
        }

        _goal8_pos << transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z();
        _goal8_or << transform.getRotation().w(),  transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z();
        r.sleep();
    }    
}


/////////////////////////////////////////////////////////////////////////////////


void TF_NAV::Goal( Eigen::Vector3d& goal_pos,  Eigen::Vector4d& goal_or, int goal_number) {
    
    move_base_msgs::MoveBaseGoal goal;

    MoveBaseClient ac("move_base", true);
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up"); 
    }

    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = goal_pos[0];
    goal.target_pose.pose.position.y = goal_pos[1];
    goal.target_pose.pose.position.z = goal_pos[2];
    goal.target_pose.pose.orientation.w = goal_or[0];
    goal.target_pose.pose.orientation.x = goal_or[1];
    goal.target_pose.pose.orientation.y = goal_or[2];
    goal.target_pose.pose.orientation.z = goal_or[3];
    
    ROS_INFO("Sending Goal %d", goal_number);
    ac.sendGoal(goal);
    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("The mobile robot arrived at the Goal %d", goal_number);
         }

    else{
        ROS_INFO("The base failed to move for some reason");
    }
    }





void TF_NAV::send_goal() {
    ros::Rate r( 5 );
    int cmd;
    int count;
    move_base_msgs::MoveBaseGoal goal;

    while ( ros::ok() )
    { 

        std::cout<<"\nInsert 1 to set the order: Goal3 ----> Goal4 ----> Goal2 -----> Goal1 "<<std::endl;
        std::cout<<"\nInsert 2 to set the order: Goal5 ----> Goal6 ----> Goal7 "<<std::endl;
        std::cout<<"\nInsert 3 to send the robot in the proximity of obstacle 9 (Goal8) "<<std::endl;


        std::cin>>cmd;

        if ( cmd == 1) {
        
        //2.c)
        TF_NAV::Goal(_goal3_pos,_goal3_or,3);
        TF_NAV::Goal(_goal4_pos,_goal4_or,4);
        TF_NAV::Goal(_goal2_pos,_goal2_or,2);
        TF_NAV::Goal(_goal1_pos,_goal1_or,1);
            
        }
        else if (cmd ==2){

        //3.a)
        TF_NAV::Goal(_goal5_pos,_goal5_or,5);
        TF_NAV::Goal(_goal6_pos,_goal6_or,6);
        TF_NAV::Goal(_goal7_pos,_goal7_or,7);

        }
        else if (cmd ==3){
        
        //4.b)
        TF_NAV::Goal(_goal3_pos,_goal3_or,3);
        TF_NAV::Goal(_goal8_pos,_goal8_or,8);
        std::cout<<"\n new goal with x=x_m +1 , y=ym "<<std::endl;

         move_base_msgs::MoveBaseGoal goal;

        MoveBaseClient ac("move_base", true);
        while(!ac.waitForServer(ros::Duration(5.0))){
            ROS_INFO("Waiting for the move_base action server to come up"); 
        }

        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();

        goal.target_pose.pose.position.x = aruco_pose[0]+1;
        goal.target_pose.pose.position.y = aruco_pose[1];
        goal.target_pose.pose.position.z = _goal8_pos[2];

        goal.target_pose.pose.orientation.w = _goal8_or[0];
        goal.target_pose.pose.orientation.x = _goal8_or[1];
        goal.target_pose.pose.orientation.y = _goal8_or[2];
        goal.target_pose.pose.orientation.z = _goal8_or[3];
    
        ROS_INFO("Sending the new Goal!");
        ac.sendGoal(goal);
        ac.waitForResult();

        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("The mobile robot has reached the new goal!");
        }
        
        else{
        ROS_INFO("The base failed to move for some reason");
        }


        }
         else {
            ROS_INFO("Wrong input!");
        }
        r.sleep();
    }
    
}

void TF_NAV::run() {
    boost::thread tf_listener_fun_t( &TF_NAV::tf_listener_fun, this );
    
    
    boost::thread tf_listener_goal1_t( &TF_NAV::goal_listener_1, this );
    boost::thread tf_listener_goal2_t( &TF_NAV::goal_listener_2, this ); 
    boost::thread tf_listener_goal3_t( &TF_NAV::goal_listener_3, this );
    boost::thread tf_listener_goal4_t( &TF_NAV::goal_listener_4, this );

    boost::thread tf_listener_goal5_t( &TF_NAV::goal_listener_5, this );
    boost::thread tf_listener_goal6_t( &TF_NAV::goal_listener_6, this );
    boost::thread tf_listener_goal7_t( &TF_NAV::goal_listener_7, this ); 

    boost::thread tf_listener_goal8_t( &TF_NAV::goal_listener_8, this );

    
    boost::thread send_goal_t( &TF_NAV::send_goal, this );
    ros::spin();
}



int main( int argc, char** argv ) {
    ros::init(argc, argv, "tf_navigation");
    ros::NodeHandle n;
    ros::Subscriber aruco_sub = n.subscribe("/aruco_single/pose", 1, aruco_call);
    ros::Subscriber aruco_broadcast_sub= n.subscribe("/aruco_single/pose", 1, poseCallback);
    TF_NAV tfnav;
    tfnav.run();

    return 0;
}