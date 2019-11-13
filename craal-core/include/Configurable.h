#ifndef CRAAL_CONFIGURABLE_H_
#define CRAAL_CONFIGURABLE_H_

#include "CraalIncludes.h"

namespace craal {

/**
 * The Configurable class. A class that inherits from this gains an additional interface
 * to selected attributes. These attributes are associated to labels (std::string)
 * which then identify them. It thus becomes possible to set/get the values of these
 * attributes with std::string equivalents. This is useful when assigning attributes
 * from parts of the code which do not know in advance what attributes are available
 * and what their type is. A class that inherits from this automatically gets an
 * std::string attribute "name".
 * 
 * This class introduces the @ref Configurable::Type subclass which defines how an
 * std::string translates to an attribute's type as well as implementations for four
 * common attribute types: bool, int, float and std::string.
 */
class Configurable
{
public:
	/**
	 * The base Type class, virtual. Used to translate an std::string into an attribute's
	 * type and vice-versa.
	 */
	class Type
	{
	public:
		/**
		 * Translation: std::string => type of the value
		 * @param s_str label of the attribute
		 * @param s_ptr void pointer to the attribute's value
		 */
		virtual void assign(std::string s_str, void * s_ptr) = 0;
		/**
		 * Translation: type of the attribute => std::string
		 * @param s_ptr void pointer to the attribute's value
		 * @return attribute's value as std::string
		 */
		virtual std::string get(void * s_ptr) = 0;
	};
	
	/**
	 * The boolean-type implementation of the @ref Configurable::Type class.
	 * Translates a boolean into an std::string and vice-versa:
	 *  - a value of 1 translates as "true"
	 *  - a value of 0 translates as "false"
	 */
	class Type_bool : public Type
	{
	public:
		/**
		 * Translation: std::string => bool
		 * @param s_str label of the attribute
		 * @param s_ptr void pointer to the bool value
		 */
		virtual void assign(std::string s_str, void * s_ptr)
		{
			if (s_str.compare("true") == 0) *((bool*)s_ptr) = true;
			else *((bool*)s_ptr) = false;
		};
		/**
		 * Translation: bool => std::string
		 * @param s_ptr void pointer to the bool value
		 * @return bool value as std::string
		 */
		virtual std::string get(void * s_ptr)
		{
			if (*((bool*)s_ptr)) return "true";
			else return "false";
		};
	};
	
	/**
	 * The integer-type implementation of the @ref Configurable::Type class.
	 * Translates an integer into an std::string and vice-versa.
	 */
	class Type_int : public Type
	{
	public:
		/**
		 * Translation: std::string => int
		 * @param s_str label of the attribute
		 * @param s_ptr void pointer to the int value
		 */
		virtual void assign(std::string s_str, void * s_ptr)
		{
			*((int*)s_ptr) = atoi(s_str.c_str());
		};
		/**
		 * Translation: int => std::string
		 * @param s_ptr void pointer to the int value
		 * @return int value as std::string
		 */
		virtual std::string get(void * s_ptr)
		{
			std::stringstream ss;
			ss<< *((int*)s_ptr);
			return ss.str();
		};
	};
	
	/**
	 * The float-type implementation of the @ref Configurable::Type class.
	 * Translates a float into an std::string and vice-versa.
	 */
	class Type_float : public Type
	{
	public:
		/**
		 * Translation: std::string => float
		 * @param s_str label of the attribute
		 * @param s_ptr void pointer to the float value
		 */
		virtual void assign(std::string s_str, void * s_ptr)
		{
			*((float*)s_ptr) = atof(s_str.c_str());
		};
		/**
		 * Translation: float => std::string
		 * @param s_ptr void pointer to the float value
		 * @return float value as std::string
		 */
		virtual std::string get(void * s_ptr)
		{
			std::stringstream ss;
			ss<< *((float*)s_ptr);
			return ss.str();
		};
	};
	
	/**
	 * The std::string-type implementation of the @ref Configurable::Type class.
	 * Translates an std::string into an std::string and vice-versa.
	 */
	class Type_string : public Type
	{
	public:
		/**
		 * Translation: std::string => std::string
		 * @param s_str label of the attribute
		 * @param s_ptr void pointer to the std::string value
		 */
		virtual void assign(std::string s_str, void * s_ptr)
		{
			*((std::string*)s_ptr) = s_str;
		};
		/**
		 * Translation: std::string => std::string
		 * @param s_ptr void pointer to the std::string value
		 * @return std::string value as std::string
		 */
		virtual std::string get(void * s_ptr)
		{
			return *((std::string*)s_ptr);
		};
	};

protected:
	/**
	 * Internal representation of a configurable attribute.
	 */
	class Value
	{
	public:
		/** Boolean indicating if the attribute was recently changed. */
		bool changed;
		/** Points to an instance of @ref Configurable::Type corresponding to the attribute's type. */
		Type * type;
		/** Points to the value of the attribute. */
		void * ptr;
		
		/** Constructor. */
		Value() : changed(false), type(0), ptr(0) {};
		/** Constructor. */
		Value(Type * s_type, void * s_ptr) : changed(false), type(s_type), ptr(s_ptr) {};
		/** Destructor. */
		virtual ~Value() {if (type) delete type;};
	};
	
	/** List of configurable attributes' labels. */
	std::vector<std::string> g_configurables;
	/** Map: attribute's label <-> attribute's value (@ref Configurable::Value). */
	std::map<std::string, Value> g_configurablesMap;
	
	/** Indicates if any attribute was recently changed. */
	bool g_needsUpdate;

public:
	/** The name attribute of the object. */
	std::string name;
	/** Constructor. */
	Configurable();
	/** Destructor. */
	~Configurable();
	
	/** Clears the list of configurable attributes. */
	void clearConfigurables();
	/**
	 * Selects an attribute as configurable.
	 * @param s_name the label by which the attribute should be identified
	 * @param stype the @ref Configurable::Type type of the attribute
	 * @param s_ptr a void pointer to the value of the attribute
	 */
	void addConfigurable(std::string s_name, Type * s_type, void * s_ptr);
	/**
	 * Set an attribute's value.
	 * @param s_name the label of the attribute
	 * @param s_value the std::string value of the attribute
	 */
	void config(std::string s_name, std::string s_value);
	/**
	 * Check if an attribute has been recently changed (since the last call to
	 * @ref Configurable::update()).
	 * @param s_name the label of the attribute
	 * @return boolean if the attribute has been recently changed
	 */
	bool changed(std::string s_name);
	/**
	 * Take into account new attribute values. This method is called to allow
	 * a child class to update itself to take into account the new values of
	 * the configurable attributes if it so requires (e.g. open a file after
	 * the attribute containing its filepath was changed).
	 * It further calls @ref Configurable::updateSpecific().
	 */
	void update();
	/**
	 * Child class-defined update. Child classes need to implement this method
	 * to update themselves to take into account new attribute values.
	 * To update, call @ref Configurable::update().
	 */
	virtual void updateSpecific();
	
	/**
	 * Used to get a list of configurable attributes' labels.
	 * @return std::vector containing the std::string labels of all
	 * configurable attributes
	 */
	const std::vector<std::string> & getConfigurables() const;
	/**
	 * Used to get the value of an attribute.
	 * @param s_name the label of the attribute
	 * @return std::string equivalent of the attribute's value
	 */
	std::string getConfigurableValue(std::string s_name);
	/**
	 * Checks if a configurable attribute exists.
	 * @param s_name label of the attribute
	 * @return bool indicating if attribute "s_name" exists
	 */
	bool existsConfigurable(std::string s_name);
};

}

#endif /* CRAAL_CONFIGURABLE_H_ */
