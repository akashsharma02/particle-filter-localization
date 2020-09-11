include (${CMAKE_CURRENT_LIST_DIR}/CPM.cmake)

CPMFindPackage(
        NAME spdlog
        GITHUB_REPOSITORY gabime/spdlog
        VERSION 1.7.0
)
