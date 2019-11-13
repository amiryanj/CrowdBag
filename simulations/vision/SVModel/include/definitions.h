/**
 * @file definitions.h
 * @author Teofilo Dutra
 */

#ifndef _DEFINITIONS_H_
#define _DEFINITIONS_H_

#include <limits.h>
#include "glm.hpp"
#include "ext.hpp"

#define MFLOAT_MAX (numeric_limits<float>::max())
#define MFLOAT_MIN (numeric_limits<float>::min())
#define MDOUBLE_MAX (numeric_limits<double>::max())
#define MSIZE_T_MAX (numeric_limits<unsigned int>::max())

#define SAFE_DELETE(p) if (p) { delete p; p = NULL; }
#define SAFE_ARRAY_DELETE(p) if (p) { delete [] p; p = NULL; }

#define M_OFFSET(a,b) reinterpret_cast<void*>(offsetof(a, b))

typedef float mFloat;
typedef double mDouble;
typedef unsigned int mSize_t;
typedef glm::vec4 vec4;
typedef glm::vec3 vec3;
typedef glm::vec2 vec2;
typedef glm::uvec2 uvec2;
typedef glm::mat4 mat4;
typedef glm::mat3x2 mat3x2;

#endif
