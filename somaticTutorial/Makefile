all: server client
server: server.cpp
	g++ server.cpp -o server -lsomatic -lamino -lach
client: client.cpp
	g++ client.cpp -o client -lsomatic -lamino -lach
clean:
	rm server client
