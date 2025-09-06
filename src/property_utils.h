#pragma once

// Getter/Setter macros for properties
// NOTE: properties must be returned by value, not reference
#define PROPERTY_GET(m_type, m_prop) \
	m_type get_##m_prop() const { return _##m_prop; }
#define PROPERTY_SET(m_type, m_prop, m_update) \
	void set_##m_prop(const m_type &val) {     \
		if (val != _##m_prop) {                \
			_##m_prop = val;                   \
			m_update;                          \
		}                                      \
	}
#define PROPERTY_GET_SET(m_type, m_prop, m_update) \
	PROPERTY_GET(m_type, m_prop)                   \
	PROPERTY_SET(m_type, m_prop, m_update)

#define STR(x) #x
#define EXPORT_PROPERTY(m_type, m_property)                                                    \
	ClassDB::bind_method(D_METHOD(STR(set_##m_property), STR(m_property)), &set_##m_property); \
	ClassDB::bind_method(D_METHOD(STR(get_##m_property)), &get_##m_property);                  \
	ADD_PROPERTY(PropertyInfo(m_type, #m_property), STR(set_##m_property), STR(get_##m_property))