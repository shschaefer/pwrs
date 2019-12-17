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

#include "ros/ros.h"
#include "ros/console.h"
#include "serializer.h"

using namespace ApplicationInsights::core;

DiagnosticsSerializer::DiagnosticsSerializer() :
  nodeHandle("~")
{ 
  // Determine logging mode
  nodeHandle.param(std::string("telemetryMode"), telemetryMode, telemetryMode);
  
  // Setup the AI Telemetry Client
  if(telemetryMode)
  {
    std::string aiKey;
    nodeHandle.param(std::string("aiKey"), aiKey, std::string("10c298b6-c156-4c60-a123-373c028894e1"));
    std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>> converter;
    std::wstring aiwKey = converter.from_bytes(aiKey);
    uploader = new TelemetryClient(aiwKey);
  }
  else
  {
	 // TODO: find logging folder
     bag.open("telemetry.bag", rosbag::bagmode::Append);
  }
  
  // Listen for diagnostics events
  subcriber = nodeHandle.subscribe("/diagnostics", 1000, &DiagnosticsSerializer::DiagnosticsCallback, this);
}

DiagnosticsSerializer::~DiagnosticsSerializer() 
{ 
  if (uploader != NULL) delete(uploader);
  else bag.close();
}

void DiagnosticsSerializer::DiagnosticsCallback(const diagnostic_msgs::DiagnosticArray::ConstPtr& diag_msg)
{
  boost::mutex::scoped_lock lock(dataLock);
  for (unsigned int msg = 0; msg < diag_msg->status.size(); ++msg)
  {
    boost::shared_ptr<const diagnostic_msgs::DiagnosticStatus> event(&diag_msg->status[msg]);
	diagnosticEvents.push_back(event);
  }
}

void DiagnosticsSerializer::PublishData()
{
  {
    boost::mutex::scoped_lock lock(dataLock);
	if (!diagnosticEvents.empty())
	{
      std::vector<boost::shared_ptr<const diagnostic_msgs::DiagnosticStatus>>::iterator it;
      for(it = diagnosticEvents.begin(); it < diagnosticEvents.end(); ++it) 
	  {
		if (telemetryMode)
          uploader->TrackEvent(GetStatusNameW(it->get()), GetStatusPropertiesW(it->get()));
	    else
		  bag.write("diagnostics", ros::Time::now(), *(it->get()));
	  }
	  
	  diagnosticEvents.clear();
	}
  }
  
  uploader->Flush();
}

std::wstring DiagnosticsSerializer::GetStatusNameW(const diagnostic_msgs::DiagnosticStatus *statusItem)
{
  std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>> converter;

  return std::wstring(converter.from_bytes(statusItem->name));
}

std::string DiagnosticsSerializer::GetStatusName(const diagnostic_msgs::DiagnosticStatus *statusItem)
{
  return std::string(statusItem->name);
}

std::map<std::wstring, std::wstring> DiagnosticsSerializer::GetStatusPropertiesW(const diagnostic_msgs::DiagnosticStatus *statusItem)
{
  std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>> converter;
  std::map<std::wstring, std::wstring> props;
  
  props.insert(std::pair<std::wstring,std::wstring>(std::wstring(L"Level"), valToWMsg(statusItem->level)));
  props.insert(std::pair<std::wstring,std::wstring>(std::wstring(L"Message"), converter.from_bytes(statusItem->message)));
  props.insert(std::pair<std::wstring,std::wstring>(std::wstring(L"HWID"), converter.from_bytes(statusItem->hardware_id)));
  for (unsigned int i = 0; i < statusItem->values.size(); ++i)
  {
    props.insert(std::pair<std::wstring,std::wstring>(converter.from_bytes(statusItem->values[i].key),
				converter.from_bytes(statusItem->values[i].value)));
  }
  
  return props;  
}

std::map<std::string, std::string> DiagnosticsSerializer::GetStatusProperties(const diagnostic_msgs::DiagnosticStatus *statusItem)
{
  std::map<std::string, std::string> props;
  
  props.insert(std::pair<std::string,std::string>(std::string("Level"), valToMsg(statusItem->level)));
  props.insert(std::pair<std::string,std::string>(std::string("Message"), statusItem->message));
  props.insert(std::pair<std::string,std::string>(std::string("HWID"), statusItem->hardware_id));
  for (unsigned int i = 0; i < statusItem->values.size(); ++i)
  {
    props.insert(std::pair<std::string,std::string>(statusItem->values[i].key,
				statusItem->values[i].value));
  }
  
  return props;  
}

std::string DiagnosticsSerializer::GetStatusAsString(const diagnostic_msgs::DiagnosticStatus *statusItem)
{
  std::string status = "[" + valToMsg(statusItem->level) + "] ";

  status += statusItem->message;

  if (statusItem->values.size() > 0)
    status += " { ";
  
  bool first = true;
  for (unsigned int i = 0; i < statusItem->values.size(); ++i)
  {
	if (!first)
	  status += ", ";
    status += """" + statusItem->values[i].key + """ : ";
	status += """" + statusItem->values[i].value + """";
	first = false;
  }
  
  status += " }";

  return status;
}