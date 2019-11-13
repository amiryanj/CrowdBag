#ifndef CRAAL_CONFIGURATIONPARSER_H_
#define CRAAL_CONFIGURATIONPARSER_H_

#include "CraalIncludes.h"

namespace craal {

/**
 * The ConfigurationParser class which is helpful when loading configration files.
 * 
 * To use:
 *  - call @ref ConfigurationParser::parse() with the wanted file
 *  - call any other method to access the variables' values
 * 
 * The general syntax is as follows:
 * 
 * @code
 * 
 * # comment
 * 
 * %a_variable
 *     value1
 *     value2
 *     and_so_on
 * 
 * %another_variable value1 value2 and_so_on
 * 
 * 
 * @endcode
 */
class ConfigurationParser
{
private:
	std::map<std::string, std::vector<std::string> > g_tokens;
	
public:
	/** Constructor. */
	ConfigurationParser();
	/** Destructor. */
	virtual ~ConfigurationParser();
	
	/**
	 * Parse a file.
	 * @param s_filename path to the file
	 */
	void parse(const char * s_filename);
	
	/**
	 * Get a float value for a given variable.
	 * @param s_param name of the variable
	 * @param s_index index of the value
	 * @return value
	 */
	float getFloatValue(std::string s_param, int s_index = 0);
	/**
	 * Get an integer value for a given variable.
	 * @param s_param name of the variable
	 * @param s_index index of the value
	 * @return value
	 */
	int getIntValue(std::string s_param, int s_index = 0);
	/**
	 * Get a character string value for a given variable.
	 * @param s_param name of the variable
	 * @param s_index index of the value
	 * @return value
	 */
	const char * getStringValue(std::string s_param, int s_index = 0);
	
	/**
	 * Returns the number of available variables.
	 */
	int getNumberOfFields();
	/**
	 * Returns the name of a certain variable.
	 * @param s_index index of the variable
	 * @return the name of the variable
	 */
	const char * getFieldName(int s_index = 0);
	
	/**
	 * Returns the number of available values for a given variable.
	 * @param name of the variable
	 * @return number of values
	 */
	int getNumberOfValues(std::string s_param);
	/**
	 * Check if a given variable exists.
	 * @param s_param name of the variable
	 * @return true if variable exists, false otherwise
	 */
	bool exists(std::string s_param);
};

}

#endif /* CRAAL_CONFIGURATIONPARSER_H_ */
