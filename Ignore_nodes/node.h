#ifndef _node_h
#define _node_h

class Node_c {

  private:
    float* colorInf;
    long location[2] = {0, 0};
    float angle = 0;

  public:
    Node_c() {

    }
    /*
      Node_c(float sensorInf[], long x, long y) {
      setNode(sensorInf, x, y);
      };*/
    Node_c(long x, long y, float _angle) {
      location[0] = x;
      location[1] = y;
      angle = _angle;
    };
    void setNode(float sensorInf[], long x, long y);
    float* getNodeColor();
    long* getNodeLoc();
    float getNodeAngle();

};

void Node_c::setNode(float sensorInf[], long x, long y) {
  //memcpy(newArray, myArray2, sizeof(myArray2));
  colorInf = sensorInf;
  location[0] = x;
  location[1] = y;
}



float * Node_c::getNodeColor() {
  return colorInf;
}

long* Node_c::getNodeLoc() {
  return location;
}

float Node_c::getNodeAngle() {
  return angle;
}

#endif
