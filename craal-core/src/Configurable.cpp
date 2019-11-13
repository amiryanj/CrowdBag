#include "../include/Configurable.h"

namespace craal {

Configurable::Configurable() :
	g_needsUpdate(false)
{}

Configurable::~Configurable()
{}

void Configurable::clearConfigurables()
{
	g_configurables.clear();
	g_configurablesMap.clear();
}

void Configurable::addConfigurable(std::string s_name, Type * s_type, void * s_ptr)
{
	g_configurables.push_back(s_name);
	g_configurablesMap[s_name] = Value();
	
	g_configurablesMap[s_name].type = s_type;
	g_configurablesMap[s_name].ptr = s_ptr;
}

void Configurable::config(std::string s_name, std::string s_value)
{
	g_configurablesMap[s_name].type->assign(s_value, g_configurablesMap[s_name].ptr);
	g_configurablesMap[s_name].changed = true;
	g_needsUpdate = true;
}

bool Configurable::changed(std::string s_name)
{
	return g_configurablesMap[s_name].changed;
}

void Configurable::update()
{
	updateSpecific();
	
	g_needsUpdate = false;
	for (int i = 0; i < g_configurables.size(); i++) {
		g_configurablesMap[g_configurables[i]].changed = false;
	}
}

const std::vector<std::string> & Configurable::getConfigurables() const
{
	return g_configurables;
}

std::string Configurable::getConfigurableValue(std::string s_name)
{
	return g_configurablesMap[s_name].type->get(g_configurablesMap[s_name].ptr);
}

bool Configurable::existsConfigurable(std::string s_name)
{
	return g_configurablesMap.find(s_name) != g_configurablesMap.end();
}

void Configurable::updateSpecific()
{}

}
