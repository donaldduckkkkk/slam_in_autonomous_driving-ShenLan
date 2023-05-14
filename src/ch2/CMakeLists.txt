add_executable(motion motion.cc)
add_executable(motion_you motion_you.cc)
target_link_libraries(motion_you
        ${PROJECT_NAME}.common
        ${PROJECT_NAME}.tools
        )
target_link_libraries(motion
${PROJECT_NAME}.common
${PROJECT_NAME}.tools
)
        
