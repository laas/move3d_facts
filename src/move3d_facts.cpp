#include <iostream>
#include <string>
#include <vector>

#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>

#include <toaster_msgs/Fact.h>
#include <toaster_msgs/FactList.h>
#include <toaster_msgs/ObjectListStamped.h>
#include <toaster_msgs/RobotListStamped.h>
#include <toaster_msgs/HumanListStamped.h>

#include <boost/tuple/tuple.hpp>
#include <boost/foreach.hpp>

#include <libmove3d/planners/API/project.hpp>
#include <libmove3d/planners/API/FactsAndProperties/factManager.hpp>
#include <libmove3d/planners/API/FactsAndProperties/propertyManager.hpp>
#include <libmove3d/planners/GTP/GTPTools/worldState.hpp>
#include <libmove3d/planners/Logging/Logger.h>

#include <tr1/shared_ptr.h>

#include <move3d_ros_lib/scenemanager.h>
#include <move3d_ros_lib/savescenariosrv.h>

#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>

#define foreach BOOST_FOREACH

using namespace std;
using namespace move3d;

typedef tr1::shared_ptr<WorldState> WorldState_p;
using namespace toaster_msgs;

class Move3dFacts
{
    Move3dFacts();
public:
    static Move3dFacts *getInstance();
    virtual ~Move3dFacts();

    bool init(ros::NodeHandle *nh);
    void initSubscriber();
    int run();

    bool enableSrvCB(std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &resp);
    static void worldUpdateCB(const ObjectListStampedConstPtr &object_list, const HumanListStampedConstPtr &human_list, const RobotListStampedConstPtr &robot_list);
    void updateAndCompute(const ObjectListStampedConstPtr &object_list, const HumanListStampedConstPtr &human_list, const RobotListStampedConstPtr &robot_list);
    void updateEnv(const ObjectListStampedConstPtr &object_list, const HumanListStampedConstPtr &human_list, const RobotListStampedConstPtr &robot_list);

    bool computeFacts();
    bool computeFacts(std_srvs::TriggerRequest &req,std_srvs::TriggerResponse &resp);

private:
    static Move3dFacts *_instance;
    FactManager *fact_mgr;
    ros::Publisher facts_pub;
    ros::ServiceServer compute_srv;
    ros::ServiceServer enable_srv;
    ros::NodeHandle *nh;
    typedef message_filters::sync_policies::ExactTime<ObjectListStamped,HumanListStamped,RobotListStamped> SyncPolicy;
    message_filters::Subscriber<toaster_msgs::ObjectListStamped> *object_sub;
    message_filters::Subscriber<toaster_msgs::HumanListStamped>  *human_sub;
    message_filters::Subscriber<toaster_msgs::RobotListStamped>  *robots_sub;

    message_filters::Synchronizer<SyncPolicy> *sync;
    SceneManager *scMgr;

    SaveScenarioSrv *saveScenarioSrv;
};

Move3dFacts *Move3dFacts::_instance = new Move3dFacts();

Move3dFacts::Move3dFacts():
    sync(0)
{
    FactManager *fact_mgr = new FactManager();

}

Move3dFacts *Move3dFacts::getInstance()
{
    return _instance;
}

Move3dFacts::~Move3dFacts()
{
    delete sync;
    delete object_sub;
    delete human_sub;
    delete robots_sub;
    delete saveScenarioSrv;

}

bool Move3dFacts::init(ros::NodeHandle *nh)
{
    this->nh=nh;
    facts_pub = nh->advertise<toaster_msgs::FactList>("move3d_facts/factList", 1000);

    enable_srv = nh->advertiseService("/move3d_facts/enable",&Move3dFacts::enableSrvCB,this);
    compute_srv = nh->advertiseService("/move3d_facts/computeOnce",&Move3dFacts::computeFacts,this);

    scMgr = new SceneManager(nh);
    scMgr->fetchDofCorrespParam("/move3d/dof_name_corresp/PR2_ROBOT","PR2_ROBOT");

    //initSubscriber();

    //init move3d
    logm3d::initializePlannerLogger();

    std::string p3d_file,sce_file;
    nh->getParam("/move3d_facts/p3dFile",p3d_file);
    nh->getParam("/move3d_facts/sceFile",sce_file);
    if(p3d_file.size()){
        scMgr->setP3dPath(p3d_file);
        scMgr->setScePath(sce_file);
        scMgr->addModule("Entities");
        scMgr->addModule("HriInfo");
        scMgr->createScene();
    }else{
        ROS_FATAL("no /move3d_facts/p3dFile param is set");
        return false;
    }


    saveScenarioSrv = new SaveScenarioSrv(scMgr,nh);
    saveScenarioSrv->advertise("/move3d_facts/save_scenario");
    return true;
}

void Move3dFacts::initSubscriber()
{
    // human_sub = new message_filters::Subscriber<toaster_msgs::HumanListStamped>  (*nh,"/pdg/humanList",1);
    // robots_sub= new message_filters::Subscriber<toaster_msgs::RobotListStamped>  (*nh,"/pdg/robotList",1);

    // sync = new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(10),*human_sub,*robots_sub);
    // sync->registerCallback(boost::bind(&worldUpdateCB,_1,_2));

    object_sub= new message_filters::Subscriber<toaster_msgs::ObjectListStamped> (*nh,"/pdg/objectList",1);
    human_sub = new message_filters::Subscriber<toaster_msgs::HumanListStamped>  (*nh,"/pdg/humanList",1);
    robots_sub= new message_filters::Subscriber<toaster_msgs::RobotListStamped>  (*nh,"/pdg/robotList",1);

    sync = new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(10),*object_sub,*human_sub,*robots_sub);
    sync->registerCallback(boost::bind(&worldUpdateCB,_1,_2,_3));
}

int Move3dFacts::run()
{
    ros::Rate loop_rate(30);
    ros::spin();
    // while(nh->ok()){
    //     ros::spinOnce();
    //     loop_rate.sleep();
    // }
    return 0;
}

bool Move3dFacts::enableSrvCB(std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &resp)
{
    if(req.data && (!sync)){
        //enable
        initSubscriber();
    }else if (req.data == false && sync){
        //disable
        delete sync;
        sync=NULL;
        if (human_sub)
            delete human_sub;
        human_sub = NULL;
        if (robots_sub)
            delete robots_sub;
        robots_sub = NULL;
        if (object_sub)
            delete object_sub;
        object_sub = NULL;
    }
    resp.message=string("");
    resp.success=true;
    return true;
}

void Move3dFacts::worldUpdateCB(const ObjectListStampedConstPtr &object_list, const HumanListStampedConstPtr &human_list, const RobotListStampedConstPtr &robot_list){
    ROS_DEBUG("received world update");
    _instance->updateAndCompute(object_list,human_list,robot_list);
}

void Move3dFacts::updateAndCompute(const ObjectListStampedConstPtr &object_list, const HumanListStampedConstPtr &human_list, const RobotListStampedConstPtr &robot_list)
{
    this->updateEnv(object_list,human_list,robot_list);
    this->computeFacts();

}

void Move3dFacts::updateEnv(const ObjectListStampedConstPtr &object_list, const HumanListStampedConstPtr &human_list, const RobotListStampedConstPtr &robot_list)
{
    foreach(const Object &o,object_list->objectList){
        scMgr->updateObject(o.meEntity.name,o.meEntity.pose);
    }
    foreach(const toaster_msgs::Robot &r,robot_list->robotList){
        std::vector<double> q;
        for(unsigned int i=1;i<r.meAgent.skeletonJoint.size();++i){
            const toaster_msgs::Joint &jnt = r.meAgent.skeletonJoint[i];
            q.push_back(jnt.position);
        }
        bool ok = scMgr->updateRobot(r.meAgent.meEntity.id,r.meAgent.meEntity.pose,q);
        if(!ok){
            //failure, try to provide the joint list name to scMgr
            ROS_DEBUG("try to provide the joint list name to scMgr");
            std::vector<std::string> names;
            for(unsigned int i=1;i<r.meAgent.skeletonNames.size();++i){
                const std::string &name = r.meAgent.skeletonNames[i];
                names.push_back(name);
            }
            scMgr->setDofNameOrdered(r.meAgent.meEntity.name,names);
            scMgr->updateRobot(r.meAgent.meEntity.id,r.meAgent.meEntity.pose,q);
        }
    }
    if(human_list->humanList.size()){
        ROS_WARN_ONCE("No support for humans yet. Cannot update human position");
        foreach (const toaster_msgs::Human &h, human_list->humanList) {
            ROS_DEBUG("Update human %s (%s)",h.meAgent.meEntity.name.c_str(),h.meAgent.meEntity.id.c_str());
            std::map<std::string, geometry_msgs::Pose> joints;
            for(uint i=0;i<h.meAgent.skeletonNames.size();++i){
                joints[h.meAgent.skeletonNames[i]]=h.meAgent.skeletonJoint[i].meEntity.pose;
            }
            scMgr->updateHuman(h.meAgent.meEntity.id,h.meAgent.meEntity.pose,joints);
        }
    }
}

bool Move3dFacts::computeFacts(std_srvs::TriggerRequest &req,std_srvs::TriggerResponse &resp){
    resp.success=this->computeFacts();
    // for debug:
    // g3d_set_win_camera(G3D_WIN->vs,4,5,0,20,0.0,1.2,0,0,1);
    // g3d_save_win_camera(G3D_WIN->vs);
    // g3d_draw_allwin_active();
    // resp.success = true;
    resp.message = std::string("");
    return true;
}

bool Move3dFacts::computeFacts(){
    toaster_msgs::FactList factList_msg;
    toaster_msgs::Fact fact_msg;

    std::map<std::string,Facts::FactType> factTypeMap=FactTypeTranslation::getInstance()->getFactTypeMap();
    typedef pair<string,Facts::FactType> StringFactPair_t;

    bool updateBaseOnly(0);
    nh->param("/move3d_facts/update_base_only",updateBaseOnly,false);
    scMgr->setUpdateAcceptBaseOnly(updateBaseOnly);


    WorldState_p ws(new WorldState(global_Project->getActiveScene()));
    ws->saveAll();
    uint facts_computed_nb(0);
    //for all existing fact types, get if we need to compute it, and compute
    foreach(StringFactPair_t it,factTypeMap){
        std::string &k=it.first;
        Facts::FactType &v=it.second;
        bool compute;
        //do not compute by default
        nh->param("/move3d_facts/enable_facts/"+k,compute,false);

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
                facts_computed_nb++;
            }
        }

        if(!nh->ok())
            break;
    }

    ROS_DEBUG("computed %u facts",facts_computed_nb);

    //fact_mgr->computeAllEnabledFacts();
    //props->computeAllEnabledProperties();


    facts_pub.publish(factList_msg);
    factList_msg.factList.clear();
}


int main(int argc, char** argv) {

    ros::init(argc, argv, "move3d_facts");
    ros::NodeHandle node;

    Move3dFacts *core = Move3dFacts::getInstance();

    if(core->init(&node))
        core->run();

    ROS_DEBUG("exiting loop");
    return 0;
}

