#include <ros/ros.h>
#include <go_to_person/msgs/people.h>

class social_layer {
private:
    void get_map();
    void update_map();
    void get_people();
    void draw_boundaries();
    void project_direction();
public:
    ros::NodeHandle n;
}






