env.pb: env.proto
	protoc -o$@ $<

env.pb.c env.pb.h: env.pb
	python nanopb/generator/nanopb_generator.py $<
