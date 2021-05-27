// ======================================================================================
// Copyright 2017 State Key Laboratory of Remote Sensing Science, 
// Institute of Remote Sensing Science and Engineering, Beijing Normal University

// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// ======================================================================================

#ifndef _CFG_H_
#define _CFG_H_
#include <algorithm>
#include <iostream>  
#include <string>  
#include <fstream> 
#include <istream>
#include <map>
#include <utility>

class Cfg
{
private:
	std::map<std::string, std::string> m_parameters;

public:
	Cfg() = default;

	Cfg(const std::string& fileName) : m_parameters{}
	{
		std::ifstream config(fileName);
		const std::string delimiter{"="};

		if (config)
		{
			while(config)
			{
				std::string line{};
				std::getline(config, line);

				std::string key{};
				key = line.substr(0, line.find(delimiter));

				std::string value{};
				value = line.substr(line.find(delimiter) + 1);

				m_parameters.emplace(std::make_pair(key, value));
			}
		}
		else
		{
			std::cout << "Cannot open for reading: " << fileName << '\n';
		}

		if (config.is_open())
			config.close();
	}

	std::string getValue(const std::string& key)
	{
		if (m_parameters.find(key) != m_parameters.end())
		{
			return m_parameters.at(key);
		}
		return "";
	}

	bool readConfigFile(const char * cfgfilepath, const std::string & key, std::string & value)
	{
		std::fstream cfgFile;
		cfgFile.open(cfgfilepath);
		if (!cfgFile.is_open())
		{
			std::cout << "can not open cfg file!" << std::endl;
			return false;
		}
		char tmp[1000];
		while (!cfgFile.eof())
		{
			cfgFile.getline(tmp, 1000);
			std::string line(tmp);
			std::size_t pos = line.find('=');
			if (pos == std::string::npos) return false;
			std::string tmpKey = line.substr(0, pos);
			if (key == tmpKey)
			{
				value = line.substr(pos + 1);
				return true;
			}
		}
		return false;
	}
};


#endif