//#ifndef ROBOTARMPOSE_H
//#define ROBOTARMPOSE_H

//#include <string>
//#include<tuple>

//class RobotArmPose
//{
//public:
//    RobotArmPose(){
//        setName("");
//        setJoints(0, 0, 0, 0, 0, 0);
//    }

//    RobotArmPose(std::string name, float joint_1, float joint_2 = 0, float joint_3 = 0, float joint_4 = 0, float joint_5 = 0, float joint_6 = 0){
//        setName(name);
//        setJoints(joint_1, joint_2, joint_3, joint_4, joint_5, joint_6);
//    }

//    ~RobotArmPose() = default;

//    void setName(std::string name) {
//        this->name = name;
//    }

//    std::string getName(void) const {
//        return name;
//    }

//    void setJoints(float joint_1, float joint_2 = 0, float joint_3 = 0, float joint_4 = 0, float joint_5 = 0, float joint_6 = 0) {
//        this->joint_1 = joint_1;
//        this->joint_2 = joint_2;
//        this->joint_3 = joint_3;
//        this->joint_4 = joint_4;
//        this->joint_5 = joint_5;
//        this->joint_6 = joint_6;
//    }

//    std::tuple <float, float, float, float, float, float> getJoints(void) const {
//         return std::make_tuple(joint_1, joint_2, joint_3, joint_4, joint_5, joint_6);
//    }

//private:
//    std::string name;
//    float joint_1;
//    float joint_2;
//    float joint_3;
//    float joint_4;
//    float joint_5;
//    float joint_6;
//};

//#endif // ROBOTARMPOSE_H
