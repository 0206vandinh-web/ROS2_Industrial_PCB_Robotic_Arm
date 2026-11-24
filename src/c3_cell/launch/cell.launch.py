from pathlib import Path
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,ExecuteProcess,OpaqueFunction,SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory,get_package_prefix
def create(ctx):
    cfg=lambda k:LaunchConfiguration(k).perform(ctx)
    p=Path(get_package_share_directory('c3_cell'))
    sim=cfg('backend')=='sim';clock={'use_sim_time':sim}
    desc={'robot_description':ParameterValue((p/'models/arm.urdf').read_text(),value_type=str)}
    planning={'robot_description_semantic':ParameterValue((p/'config/arm.srdf').read_text(),value_type=str),
       'robot_description_kinematics':{'arm':{'kinematics_solver':'kdl_kinematics_plugin/KDLKinematicsPlugin','kinematics_solver_timeout':.1}},
       'planning_pipelines':['ompl'],'default_planning_pipeline':'ompl',
       'ompl':{'planning_plugin':'ompl_interface/OMPLPlanner','request_adapters':'default_planner_request_adapters/FixStartStateBounds',
               'planner_configs':{'RRTConnect':{'type':'geometric::RRTConnect'}},'arm':{'planner_configs':['RRTConnect']}},
       'allow_trajectory_execution':False,'publish_planning_scene':True,'publish_geometry_updates':True,'publish_state_updates':True,
       'publish_transforms_updates':True}
    def node(pkg,exe,*params,**kw):return Node(package=pkg,executable=exe,output='screen',parameters=[clock,*params],**kw)
    actions=[]
    if sim:
        actions+=[SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH',get_package_prefix('c3_cell')+'/lib:'+os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH','')),
                  ExecuteProcess(cmd=['gz','sim','-s','-r','-v','3',str(p/'models/workcell.sdf')],output='screen')]
    else:actions.append(node('c3_cell','gateway.py',{'interface':cfg('interface')}))
    actions+=[node('robot_state_publisher','robot_state_publisher',desc),
              node('moveit_ros_move_group','move_group',desc,planning),
              node('c3_cell','vision.py',{'external':not sim}),
              node('c3_cell','station.py',{'recipe':cfg('recipe'),'loops':int(cfg('loops'))})]
    if cfg('rviz')=='true':actions.append(node('rviz2','rviz2',desc,planning,arguments=['-d',str(p/'config/workcell.rviz')]))
    return actions
def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('backend',default_value='sim',choices=['sim','hardware']),
      DeclareLaunchArgument('rviz',default_value='false'),DeclareLaunchArgument('recipe',default_value='assembly'),
      DeclareLaunchArgument('loops',default_value='1'),DeclareLaunchArgument('interface',default_value='can0'),OpaqueFunction(function=create)])
