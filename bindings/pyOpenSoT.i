/* File : pyOpenSoT.i */
%module pyOpenSoT
%include std_string.i
%include std_map.i

%{
/* Note : always include headers following the inheritance order */

// OpenSoT
#include "OpenSoT/Task.h"
#include "OpenSoT/tasks/velocity/Cartesian.h"
%}

/* Note : always include headers following the inheritance order */
// OpenSoT

%include "OpenSoT/Task.h"

namespace OpenSoT {
    %template(yTask) Task< yarp::sig::Matrix,yarp::sig::Vector >;
}

%include "OpenSoT/tasks/velocity/Cartesian.h"

%feature("autodoc",3);
