#include "../include/ConfigurationParser.h"

namespace craal {

ConfigurationParser::ConfigurationParser()
{}

ConfigurationParser::~ConfigurationParser()
{}

void ConfigurationParser::parse(const char * s_fileName)
{
	std::ifstream ifs;
	ifs.open(s_fileName, std::ifstream::binary);
	std::string param;
	std::string value;
	char buffer[256];

	while (!ifs.eof()) {
		ifs>> value;

		if (value.size() == 0 || value[0] == '#') {
			ifs.getline(buffer, 256);
		}
		else if (value[0] == '%') {
			param = value.substr(1);
			g_tokens[param] = std::vector<std::string>();
		}
		else {
			g_tokens[param].push_back(value);
		}
	}

	ifs.close();
}

float ConfigurationParser::getFloatValue(std::string s_param, int s_index)
{
	return atof(g_tokens[s_param][s_index].c_str());
}

int ConfigurationParser::getIntValue(std::string s_param, int s_index)
{
	return atoi(g_tokens[s_param][s_index].c_str());
}

const char * ConfigurationParser::getStringValue(std::string s_param, int s_index)
{
	return g_tokens[s_param][s_index].c_str();
}

int ConfigurationParser::getNumberOfFields()
{
	return g_tokens.size();
}

const char * ConfigurationParser::getFieldName(int s_index)
{
	std::map<std::string, std::vector<std::string> >::iterator it;
	it = g_tokens.begin();
	for (int i = 0; i < s_index && it != g_tokens.end(); i++)it++;

	return it->first.c_str();
}

int ConfigurationParser::getNumberOfValues(std::string s_param)
{
	return g_tokens[s_param].size();
}

bool ConfigurationParser::exists(std::string s_param)
{
	if (g_tokens.find(s_param) == g_tokens.end()) {
		return false;
	}

	return true;
}

}
