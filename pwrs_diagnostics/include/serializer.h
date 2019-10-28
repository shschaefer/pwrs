/*
The MIT License (MIT)

Copyright (c) 2016 

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifndef DIAGNOSTICS_SERIALIZER_H
#define DIAGNOSTICS_SERIALIZER_H

#include <boost/thread/mutex.hpp>

#include "ros/ros.h"
#include "rosbag/bag.h"
#include "diagnostic_msgs/DiagnosticArray.h"
#include "TelemetryClient.h"

#include <locale>
#include <codecvt>

using namespace ApplicationInsights::core;

class DiagnosticsSerializer
{
  public:
    DiagnosticsSerializer();
	~DiagnosticsSerializer();
    void DiagnosticsCallback(const diagnostic_msgs::DiagnosticArray::ConstPtr& diag_msg);
    void PublishData();

  private:
    std::wstring GetStatusNameW(const diagnostic_msgs::DiagnosticStatus *statusItem);
    std::string GetStatusName(const diagnostic_msgs::DiagnosticStatus *statusItem);
    std::map<std::wstring, std::wstring> GetStatusPropertiesW(const diagnostic_msgs::DiagnosticStatus *statusItem);
    std::map<std::string, std::string> GetStatusProperties(const diagnostic_msgs::DiagnosticStatus *statusItem);
    std::string GetStatusAsString(const diagnostic_msgs::DiagnosticStatus *statusItem);

    ros::NodeHandle nodeHandle;
    ros::Subscriber subcriber;
	bool telemetryMode = true;
    TelemetryClient *uploader = NULL;
    rosbag::Bag bag;
	
    boost::mutex dataLock;
	std::vector<boost::shared_ptr<const diagnostic_msgs::DiagnosticStatus>> diagnosticEvents;
};

inline std::string valToMsg(const int val)
{
  if (val == diagnostic_msgs::DiagnosticStatus::DIAG_OK)
    return "OK";
  else if (val == diagnostic_msgs::DiagnosticStatus::DIAG_WARN)
    return "Warning";
  else if (val == diagnostic_msgs::DiagnosticStatus::DIAG_ERROR)
    return "Error";
  else if (val == diagnostic_msgs::DiagnosticStatus::DIAG_STALE)
    return "Stale";
  
  ROS_ERROR("Attempting to convert diagnostic level %d into string. Values are: {0: \"OK\", 1: \"Warning\", 2: \"Error\", 3: \"Stale\"}", val);
  return "Error";
}

inline std::wstring valToWMsg(const int val)
{
  std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>> converter;
  return converter.from_bytes(valToMsg(val));
}

#endif DIAGNOSTICS_SERIALIZER_H