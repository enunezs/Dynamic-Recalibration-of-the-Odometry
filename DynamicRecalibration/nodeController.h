#include "node.h"

class nodeController_c {
  public:
    nodeController_c() {
      node_num = 0;
      node_limit = 10;
    };
    boolean checkNode_R(long x, long y);

    boolean checkNode(Node_c node);
    Node_c getClosestNode(Node_c node);
    Node_c getEarliestNode(Node_c node);
    boolean checkJumpNode(long x, long y, Node_c node);
    boolean checkNode(float sensor_value[]);
    void addNode(float sensorInf[], long x, long y);
    void addNode(long x, long y, float angle);
    int getNodeNumber();
    Node_c getNode(int num);

  private:
    Node_c nodes[5];
    int node_num;
    int node_limit;
    long nodeController_c::getDistance(Node_c node1, Node_c node2);
    int findMin(float arrayList[], int size);

};

boolean nodeController_c::checkNode_R(long x, long y) {
  //to check whether this node is recorded
  //input: x->the x-line value; y->the y-line value;
  //output: the boolean value(return 0: never recorded; return 1: recorded)
  float err = 500;  //roughly
  if (node_num == 0) {
    return 0;
  }
  for (int i = 0; i < node_num; i++) {
    Node_c node = nodes[i];
    long* loc = node.getNodeLoc();
    if (loc[0] + err > x && loc[0] - err < x) {
      if (loc[1] + err > y && loc[1] - err < y) {
        return 1;
      }
    }
  }
  return 0;
}

boolean nodeController_c::checkJumpNode(long x, long y, Node_c node) {
  //this function is to check whether the romi is on the target node
  //input: x->the x-line value; y->the y-line value; node->the target node value
  //output: the boolean value
  float err = 1000;
  long* loc = node.getNodeLoc();
  if (loc[0] + err > x && loc[0] - err < x) {
    if (loc[1] + err > y && loc[1] - err < y) {
      return 1;
    }
  }
  return 0;
}



boolean nodeController_c::checkNode(float sensor_value[]) {
  //to check whether this is a node value
  //input: the sensor value
  //output: the boolean value
  float L_sensor = sensor_value[0];
  float C_sensor = sensor_value[1];
  float R_sensor = sensor_value[2];

  if (L_sensor < 800) {
    if (C_sensor < 800) {
      if (R_sensor < 800) {
        return 1;
      }
    }
  }
  return 0;
  //later
}

Node_c nodeController_c::getClosestNode(Node_c node) {
  //to get the closest node from the list
  //input: the target node
  //output: the closest node
  float distList[node_num];
  if (node_num == 0) {
    return node;
  }
  for (int i = 0; i < node_num; i++) {
    distList[i] = getDistance(node, nodes[i]);
  }
  int minIdex = findMin(distList, node_num);
  return nodes[minIdex];
}



void nodeController_c::addNode(float sensorInf[], long x, long y) {
  //to add a new node into the list
  //input: nessesary information ot the node_c-> x_value, y_value and the sensor information
  Node_c node(sensorInf, x, y);
  node_num = node_num + 1;
  nodes[node_num - 1] = node;
}

void nodeController_c::addNode(long x, long y, float angle) {
  //to add a new node into the list
  //input: nessesary information ot the node_c-> x_value, y_value and the sensor information
  Node_c node(x, y, angle);
  node_num = node_num + 1;
  nodes[node_num - 1] = node;
}


    


Node_c nodeController_c::getNode(int num) {
  return nodes[num];
}

long nodeController_c::getDistance(Node_c node1, Node_c node2) {
  long* loc1 = node1.getNodeLoc();
  long* loc2 = node2.getNodeLoc();
  return abs(loc1[0] - loc2[0]) + abs(loc1[1] - loc2[1]);
}

int nodeController_c::findMin(float arrayList[], int size) {
  int minIndex = 0;
  int min = arrayList[minIndex];
  for (int i = 1; i < size; i++) {
    if (min > arrayList[i]) {
      minIndex = i;
      min = arrayList[i];
    }
  }
  return minIndex;
}


int nodeController_c::getNodeNumber() {
  return node_num;
}
