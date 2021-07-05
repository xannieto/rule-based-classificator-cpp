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
	const std::string delimiter{"="};
	bool m_read;

public:
	std::string m_fileName;

	Cfg() 
	: m_parameters{}, m_read{false}, m_fileName{""}
	{
	}

	Cfg(const std::string& fileName)
	: m_parameters{}, m_fileName{fileName}, m_read{false}
	{
		readFile(m_fileName);
	}

	bool readFile(const std::string& inputFile)
	{
		if (m_read)
		{
			m_read = false;
			m_parameters.erase(m_parameters.begin(), m_parameters.end());
		}
		std::ifstream config(inputFile);

		if (config)
		{
			m_read = true;
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

		if (config.is_open())
			config.close();

		return m_read;
	}

	std::string getValue(const std::string& key)
	{
		if (m_parameters.find(key) != m_parameters.end())
		{
			return m_parameters.at(key);
		}
		return nullptr;
	}
	
	bool isGoodRead()
	{
		return m_read;
	}
};


#endif