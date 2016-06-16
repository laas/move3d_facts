#include <iostream>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <toaster_msgs/Fact.h>
#include <toaster_msgs/FactList.h>

#include <boost/tuple/tuple.hpp>
#include <boost/foreach.hpp>

#include <libmove3d/planners/API/project.hpp>
#include <libmove3d/planners/API/FactsAndProperties/factManager.hpp>
#include <libmove3d/planners/API/FactsAndProperties/propertyManager.hpp>
#include <libmove3d/planners/GTP/GTPTools/worldState.hpp>
#include <libmove3d/planners/Logging/Logger.h>

#include <tr1/shared_ptr.h>

#define foreach BOOST_FOREACH

using namespace std;

typedef tr1::shared_ptr<WorldState> WorldState_p;

int main(int argc, char** argv) {

    ros::init(argc, argv, "move3d_facts");
    ros::NodeHandle node;
    ros::Rate loop_rate(30);

    // Publishing
    ros::Publisher facts_pub = node.advertise<toaster_msgs::FactList>("move3d_facts/factList", 1000);

    toaster_msgs::FactList factList_msg;
    toaster_msgs::Fact fact_msg;

    //init move3d
    logm3d::initializePlannerLogger();

    std::string p3d_file;
    node.getParam("/move3d_facts/p3dFile",p3d_file);
    if(p3d_file.size()){
        p3d_col_set_mode(p3d_col_mode_none);
        p3d_read_desc(p3d_file.c_str());
        p3d_col_set_mode(p3d_col_mode_pqp);
        p3d_col_start(p3d_col_mode_pqp);
        global_Project = new Project(new Scene(XYZ_ENV));
        global_Project->addModule("GTP");
        global_Project->init();
    }else{
        ROS_FATAL("no /move3d_facts/p3dFile param is set");
        return 1;
    }

    /************************/
    /* Start of the Ros loop*/
    /************************/

    FactManager *fact_mgr = new FactManager();
    //PropertyManager *props=new PropertyManager();

    while (node.ok()) {
      std::map<std::string,Facts::FactType> factTypeMap=FactTypeTranslation::getInstance()->getFactTypeMap();
      typedef pair<string,Facts::FactType> StringFactPair_t;
      WorldState_p ws = WorldState_p(new WorldState(global_Project->getActiveScene()));
      ws->saveAll();
      uint facts_computed_nb(0);
      //for all existing fact types, get if we need to compute it, and compute
      foreach(StringFactPair_t it,factTypeMap){
          std::string &k=it.first;
          Facts::FactType &v=it.second;
          bool compute;
          //do not compute by default
          node.param("/move3d_facts/enable_facts/"+k,compute,false);

          if(compute){
              ROS_DEBUG("computing facts of type %s",k.c_str());
              //TODO: compute too much stuff (all possible combinations of isReachable, for every Robot)
              vector<VirtualFact*> computed_facts = fact_mgr->computeAllOneTypeFacts(v,ws.get());
              foreach(VirtualFact *fact_it,computed_facts){
                  fact_msg.property = k;
                  fact_msg.subProperty = FactTypeTranslation::getInstance()->getFactSubTypeAsString(fact_it->getDefaultSubType());
                  fact_msg.propertyType = "geometric";
                  fact_msg.subjectId = fact_it->getEntity1()->getName();
                  fact_msg.targetId = fact_it->getEntity2()->getName();
                  fact_msg.time= ros::Time::now().toNSec();
                  fact_msg.factObservability = 1.0;
                  fact_msg.confidence = 1.0;
                  fact_msg.doubleValue= fact_it->getNumericValue();
                  if(fact_it->getBoolValue()){
                      fact_msg.stringValue= "true";
                  }else{
                      fact_msg.stringValue= "false";
                  }

                  factList_msg.factList.push_back(fact_msg);
              }
          }

          if(!node.ok())
              break;
      }

      ROS_DEBUG("computed %u facts",facts_computed_nb);

      //fact_mgr->computeAllEnabledFacts();
      //props->computeAllEnabledProperties();


        facts_pub.publish(factList_msg);
        ros::spinOnce();

        loop_rate.sleep();
    }
    return 0;
}
