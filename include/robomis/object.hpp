#ifndef OBJECT_HPP
#define OBJECT_HPP

#include <string>
#include <QPixmap>
#include <QImage>

class Object
{
public:
    Object();
    Object(int object_type, int instance_id, std::string description = "", std::string image_name = "");
    ~Object(){}
    void setDescription(std::string description);
    void setColor(std::string color_name);
    void setImage(std::string image_name);
    std::string getDescription(void) { return description; }
    std::string getColor(void) { return color_name; }
    QPixmap getImage(void) { return pm; }
private:
    QImage img;
    QPixmap pm;
    int object_type;
    int instance_id;
    std::string description;
    std::string color_name;

};

#endif // OBJECT_HPP
