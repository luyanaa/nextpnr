set(SOURCES
    constids.inc
    xist.cc
    gfxids.inc
)

add_nextpnr_himbaechel_microarchitecture(${uarch}
    CORE_SOURCES ${SOURCES}
)
