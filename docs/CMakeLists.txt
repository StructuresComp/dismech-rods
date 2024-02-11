find_package(Doxygen REQUIRED)
find_package(Sphinx REQUIRED)
find_program(BREATHE_APIDOC breathe-apidoc)

if(NOT BREATHE_APIDOC)
    message(FATAL_ERROR "breathe-apidoc not found!"
                        "Please install using: pip3 install breathe")
endif()

set(DOXYGEN_INPUT_DIR ${PROJECT_SOURCE_DIR}/src)
set(DOXYGEN_OUTPUT_DIR ${CMAKE_CURRENT_BINARY_DIR}/doxygen)
set(DOXYGEN_INDEX_FILE ${DOXYGEN_OUTPUT_DIR}/xml/index.xml)
set(DOXYFILE_IN ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in)
set(DOXYFILE_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

# Set the Doxygen input and output directories in the Doxyfile
configure_file(${DOXYFILE_IN} ${DOXYFILE_OUT} @ONLY)

# Doxygen won't create this for us
file(MAKE_DIRECTORY ${DOXYGEN_OUTPUT_DIR}) 

# Only regenerate Doxygen when the Doxyfile or public headers change
add_custom_command(OUTPUT ${DOXYGEN_INDEX_FILE}
        	COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYFILE_OUT}
		MAIN_DEPENDENCY ${DOXYFILE_OUT} ${DOXYFILE_IN}
        	COMMENT "Generating docs using doxygen"
		VERBATIM)

# Nice named target for running doxygen
add_custom_target(Doxygen ALL DEPENDS ${DOXYGEN_INDEX_FILE})

# ...........................................................
# Using breathe-apidoc

set(BREATHE_INDEX_FILE ${PROJECT_SOURCE_DIR}/docs/filelist.rst)
set(BREATHE_SOURCE ${PROJECT_SOURCE_DIR}/build/docs/doxygen/xml)
set(BREATHE_OUTPUT_DIR ${PROJECT_SOURCE_DIR}/docs/breathe)
file(MAKE_DIRECTORY ${BREATHE_OUTPUT_DIR}) 

add_custom_command(OUTPUT ${BREATHE_INDEX_FILE}
	DEPENDS ${DOXYGEN_INDEX_FILE}
    COMMAND breathe-apidoc -o "${BREATHE_OUTPUT_DIR}" -f -m -T "${BREATHE_SOURCE}"
    COMMENT "Building rst files using breathe-apidoc"
	VERBATIM
)
# Nice named target for running breathe-apidoc
add_custom_target(Breathe ALL DEPENDS ${BREATHE_INDEX_FILE})

# ................................................
# Using Sphinx

set(SPHINX_SOURCE ${CMAKE_CURRENT_SOURCE_DIR})
set(SPHINX_BUILD ${CMAKE_CURRENT_BINARY_DIR}/sphinx)
set(SPHINX_INDEX_FILE ${SPHINX_BUILD}/index.html)

# Only regenerate Sphinx when:
#  - Doxygen has rerun 
#  - Our doc files have been updated
#  - The Sphinx config has been updated
add_custom_command(OUTPUT ${SPHINX_INDEX_FILE}
	COMMAND 
		${SPHINX_EXECUTABLE} -b html
		# Tell Breathe where to find the Doxygen output
		-Dbreathe_projects.DisMech=${DOXYGEN_OUTPUT_DIR}/xml
		${SPHINX_SOURCE} ${SPHINX_BUILD}
	WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
	DEPENDS 
		# Other docs files you want to track should go here (or in some variable)
		${CMAKE_CURRENT_SOURCE_DIR}/index.rst
		${DOXYGEN_INDEX_FILE}
		${CMAKE_CURRENT_SOURCE_DIR}/classlist.rst
		${CMAKE_CURRENT_SOURCE_DIR}/filelist.rst
		${CMAKE_CURRENT_SOURCE_DIR}/structlist.rst
		${CMAKE_CURRENT_SOURCE_DIR}/namespacelist.rst
	MAIN_DEPENDENCY ${SPHINX_SOURCE}/conf.py
	COMMENT "Generating documentation with Sphinx")

# Nice named target so we can run the job easily
add_custom_target(Sphinx ALL DEPENDS ${SPHINX_INDEX_FILE})

include(GNUInstallDirs)
install(DIRECTORY ${SPHINX_BUILD}
	DESTINATION ${CMAKE_INSTALL_DOCDIR})
