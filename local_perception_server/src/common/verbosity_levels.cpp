/**\file verbosity_levels.cpp
 * \brief Description...
 *
 * @version 2.0
 * @author Carlos Miguel Correia da Costa and João Pedro Carvalho de Souza
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include <local_perception_server/common/verbosity_levels.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

namespace local_perception_server {
namespace verbosity_levels {

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </imports>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <imports>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
	bool setVerbosityLevelROS(std::string level) {
		if (level == "DEBUG") {
			if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
				ros::console::notifyLoggerLevelsChanged();
			}
		} else if (level == "INFO") {
			if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info)) {
				ros::console::notifyLoggerLevelsChanged();
			}
		} else if (level == "WARN") {
			if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Warn)) {
				ros::console::notifyLoggerLevelsChanged();
			}
		} else if (level == "ERROR") {
			if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Error)) {
				ros::console::notifyLoggerLevelsChanged();
			}
		} else if (level == "FATAL") {
			if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Fatal)) {
				ros::console::notifyLoggerLevelsChanged();
			}
		} else {
			return false;
		}

		return true;
	}

	bool isROSDebug(){
		return (verifyVerbosityLevelROS() == ros::console::levels::Debug);
	}

	bool isROSWarn(){
		return (verifyVerbosityLevelROS() == ros::console::levels::Warn);
	}

	bool isROSInfo(){
		return (verifyVerbosityLevelROS() == ros::console::levels::Info);
	}

	bool isROSError(){
		return (verifyVerbosityLevelROS() == ros::console::levels::Error);
	}

	bool isROSFatal(){
		return (verifyVerbosityLevelROS() == ros::console::levels::Fatal);
	}

	ros::console::levels::Level verifyVerbosityLevelROS(){
		std::map<std::string, ros::console::levels::Level> loggers;
		ros::console::get_loggers(loggers);
		std::map<std::string, ros::console::levels::Level>::iterator it = loggers.begin();
		it++;
		return it->second;
	}

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <verbosity_levels-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
	bool setVerbosityLevelPCL(std::string level) {
		if (level == "VERBOSE") {
			pcl::console::setVerbosityLevel(pcl::console::L_VERBOSE);
		} else if (level == "DEBUG") {
			pcl::console::setVerbosityLevel(pcl::console::L_DEBUG);
		} else if (level == "INFO") {
			pcl::console::setVerbosityLevel(pcl::console::L_INFO);
		} else if (level == "WARN") {
			pcl::console::setVerbosityLevel(pcl::console::L_WARN);
		} else if (level == "ERROR") {
			pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
		} else if (level == "ALWAYS") {
			pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
		} else {
			return false;
		}

		return true;
	}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </verbosity_levels-functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


} /* namespace verbosity_levels */
} /* namespace my_action_skill */