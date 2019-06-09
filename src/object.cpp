#include "../include/robomis/object.hpp"
#include <QString>

Object::Object()
    : object_type(-1),
      instance_id(-1),
      description("")   {
}

Object::Object(int object_type, int instance_id, std::string description, std::string image_name)
    : object_type(object_type),
      instance_id(instance_id),
      description(description)   {
    setImage(image_name);
}

void Object::setDescription(std::string description) {
    this->description = description;
}

void Object::setColor(std::string color_name) {
    this->color_name = color_name;
}

void Object::setImage(std::string image_name) {
    QImage img(QString::fromStdString(image_name));
    pm = QPixmap::fromImage(img);
}
