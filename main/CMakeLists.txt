# The original firmware uses PubSubClient, which is an Arduino library rather
# than an ESP-IDF Component Registry package. Download the pinned v2.8 source
# automatically during CMake configure so the project remains easy to share.

set(PUBSUBCLIENT_DIR "${CMAKE_CURRENT_LIST_DIR}/.deps/PubSubClient")
file(MAKE_DIRECTORY "${PUBSUBCLIENT_DIR}")

set(PUBSUBCLIENT_H "${PUBSUBCLIENT_DIR}/PubSubClient.h")
set(PUBSUBCLIENT_CPP "${PUBSUBCLIENT_DIR}/PubSubClient.cpp")

if(NOT EXISTS "${PUBSUBCLIENT_H}")
    file(DOWNLOAD
        "https://raw.githubusercontent.com/knolleary/pubsubclient/v2.8/src/PubSubClient.h"
        "${PUBSUBCLIENT_H}"
        TLS_VERIFY ON
        STATUS PUBSUBCLIENT_H_STATUS
    )
    list(GET PUBSUBCLIENT_H_STATUS 0 PUBSUBCLIENT_H_CODE)
    if(NOT PUBSUBCLIENT_H_CODE EQUAL 0)
        file(REMOVE "${PUBSUBCLIENT_H}")
        message(FATAL_ERROR "Could not download PubSubClient.h. Check Internet access and try again.")
    endif()
endif()

if(NOT EXISTS "${PUBSUBCLIENT_CPP}")
    file(DOWNLOAD
        "https://raw.githubusercontent.com/knolleary/pubsubclient/v2.8/src/PubSubClient.cpp"
        "${PUBSUBCLIENT_CPP}"
        TLS_VERIFY ON
        STATUS PUBSUBCLIENT_CPP_STATUS
    )
    list(GET PUBSUBCLIENT_CPP_STATUS 0 PUBSUBCLIENT_CPP_CODE)
    if(NOT PUBSUBCLIENT_CPP_CODE EQUAL 0)
        file(REMOVE "${PUBSUBCLIENT_CPP}")
        message(FATAL_ERROR "Could not download PubSubClient.cpp. Check Internet access and try again.")
    endif()
endif()

idf_component_register(
    SRCS "main.cpp" "${PUBSUBCLIENT_CPP}"
    INCLUDE_DIRS "." "${PUBSUBCLIENT_DIR}"
    REQUIRES arduino-esp32 arduinojson
)
