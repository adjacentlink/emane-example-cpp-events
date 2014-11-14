bin_PROGRAMS = emane-example-cpp-events

BUILT_SOURCES =                      \
 antennaprofileevent.pb.cc           \
 antennaprofileevent.pb.h            \
 commeffectevent.pb.cc               \
 commeffectevent.pb.h                \
 event.pb.cc                         \
 event.pb.h                          \
 locationevent.pb.cc                 \
 locationevent.pb.h                  \
 pathlossevent.pb.cc                 \
 pathlossevent.pb.h                 

EXTRA_DIST=                          \
 antennaprofileevent.proto           \
 commeffectevent.proto               \
 event.proto                         \
 locationevent.proto                 \
 pathlossevent.proto                

emane_example_cpp_events_CXXFLAGS =  \
 $(AM_CFLAGS)                        \
 $(protobuf_CFLAGS)                  \
 $(libuuid_CFLAGS)

emane_example_cpp_events_LDFLAGS=    \
 $(protobuf_LIBS)                    \
 $(libuuid_LIBS)                     \
 -lpthread

emane_example_cpp_events_SOURCES =   \
 antennaprofileevent.pb.cc           \
 commeffectevent.pb.cc               \
 event.pb.cc                         \
 example.cc                          \
 locationevent.pb.cc                 \
 pathlossevent.pb.cc                

antennaprofileevent.pb.cc antennaprofileevent.pb.h:	antennaprofileevent.proto
	protoc -I=. --cpp_out=. $<

commeffectevent.pb.cc commeffectevent.pb.h:	commeffectevent.proto
	protoc -I=. --cpp_out=. $<

event.pb.cc event.pb.h:	event.proto
	protoc -I=. --cpp_out=. $<

locationevent.pb.cc locationevent.pb.h:	locationevent.proto
	protoc -I=. --cpp_out=. $<

pathlossevent.pb.cc pathlossevent.pb.h:	pathlossevent.proto
	protoc -I=. --cpp_out=. $<

clean-local:
	rm -f $(BUILT_SOURCES)
