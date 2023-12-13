cmake_minimum_required(VERSION 3.0.2)
project(serial_demo)



## Find required dependencies
find_package(catkin REQUIRED COMPONENTS
	  roscpp
	    serial
	    )

    ## Specify package dependencies
    catkin_package(
	      INCLUDE_DIRS 
	          include
		      /home/ti5robot/src/serial_demo/include
		        LIBRARIES 
			    serial_demo
			      CATKIN_DEPENDS 
			          roscpp serial
				    DEPENDS 
				        system_lib
					)

				## Include directories
				include_directories(
					  include
					    /home/ti5robot/src/serial_demo/include
					      ${catkin_INCLUDE_DIRS}
					      )

				      ## Specify source files
				      add_executable(serial_demo
					        /home/ti5robot/src/serial_demo/src/serial_demo.cpp
						/home/ti5robot/src/serial_demo/src/SingleCaninterface.cpp
						)

					## Specify dependencies of the executable
					add_dependencies(serial_demo
						  ${${PROJECT_NAME}_EXPORTED_TARGETS}
						    ${catkin_EXPORTED_TARGETS}
						    )

					    #link_directories(/home/ti5robot/src/serial_demo/lib)
					    ## Specify libraries to link against
					    target_link_libraries(serial_demo
						      ${catkin_LIBRARIES}
					              /home/ti5robot/controlcan/libcontrolcan.so
						      pthread
						      )

