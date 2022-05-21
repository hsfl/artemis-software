file(GLOB files "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
foreach(file ${files})
    get_filename_component(bfile ${file} NAME_WE)
        add_executable(${bfile} ${file})
        target_link_libraries(${bfile} ${HYTI_LIBS} ${COSMOS_LIBS} localzlib Library1 CosmosAgent CosmosPrint)
        install(TARGETS ${bfile} DESTINATION bin)
endforeach()

add_executable(agent_cpu "${COSMOS_SOURCE}/core/programs/agents/agent_cpu.cpp")
target_link_libraries(agent_cpu CosmosAgent CosmosDeviceDisk CosmosPrint)
install(TARGETS agent_cpu DESTINATION bin)
add_deployment_binary(agent_cpu bin)

add_executable(agent "${COSMOS_SOURCE}/core/programs/agents/agent.cpp")
target_link_libraries(agent CosmosAgent)
install(TARGETS agent DESTINATION bin)


