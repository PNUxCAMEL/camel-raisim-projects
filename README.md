# camel-raisim-projects
## How to use?
### How to add third party libraries to your project CMakeList.txt?
- Example below
```cmake
include_directories(
        "../camel-thirdparty/qpOASES/include"
)
link_directories(
        "../camel-thirdparty/qpOASES/bin"
)
```
```c++
#include <iostream>
#include <QApplication>

#include <qpOASES.hpp>

```
### Folder structures
```c++
camel-raisim-projects
|---camel-[project name]
    |---CMakeLists.txt
    |---includes
        |---camel-[project name]
            |---[project name]Main.hpp
    |---src
        |---[project name]Main.cpp
    //sub module 1
    |---[project name]_[sub folder name1]
        |---includes
            |---[project name]_[sub folder name]
                |---HeaderFile1.hpp
                |---HeaderFile2.hpp
        |---src
            |---SrcFile1.cpp
            |---SrcFile2.cpp
        |---CMakeLists.txt
    //sub module 2
    |---[project name]_[sub folder name2]
        |---includes
            |---[project name]_[sub folder name2]
                |---HeaderFile3.hpp
                |---HeaderFile4.hpp
        |---src
            |---SrcFile3.cpp
            |---SrcFile4.cpp
        |---CMakeLists.txt
```

- Branch name tip
  - if, project main branch : [Project name]
    - Ex) canine
  - if, something roll of the project : [Project name]_[roll]
    - Ex) canine_mpc
