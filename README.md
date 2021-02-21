# Roomba on the Internet
### Overview
I have been a weekly user of a Roomba for the past 15 years and absolutely love it on my hardwood floors.  I even bought one for my parents who also use it perhaps once every 2-3 weeks.  Anyhow, recently I found this old semi-functional Roomba in my basement and thought that it could be fun to use it in some kind project.  
A quick search online revealed that the Roomba has an RS-232 communication port and published communication protocols from IRobot.  After more searches, we found lots of super cool projects that people have done so this is looking more and more promising.

### Project goal
By now, we have figured out that yes the Roomba can be "hacked" and we could do something with it.  The question is what shall we do that is not just a copy-paste project from what we found on the internet.  Something that is fun but something achievable in 1-2 weeks of work.  After some thinking, here are the end-goals.

 1. Have the ability to see all the sensor data from the Roomba while it is doing a clean.
 2. Have the ability to drive the Roomba like an RC car.
 3. Do this on the internet so that the grandparents can also see it and drive the Roomba.
 4. Ok, nothing is really novel and have been done by folks in various format but I like this idea so that my kids can also be highly involved with.
 5. Have a software playground where my daughter can work with the Roomba such as writing the docking procedure, navigating from 1 room to another, fun stuff like that.

### Design
Ok, even though the work is supposed to be a collaboration of 1 adult and 2 teens, it obviously ended up with 90% of the work on my end but most of it is just the R&D work and re-usable code that would be too much for them.  They did help build the electronics, write some C++ code and write the website for the project which I am very proud of their work on that.  

### Technical
I have had experience using Raspberry Pis and Arduinos but was introduced to the ESP microcontroller from a friend and it is truly amazing especially it is easily available for $4!  

#### The Big Picture
[Picture]
 -  WebSocket is used to do bi-directional communication between the ESP to the Node server.
 - The website will received relayed real-time data using another WebSocket connection but commands from the website are sent to the Node server using an HTTP RestAPI instead.  I find this easier to test and debug using tools like Postman or browser.
 
 #####  Example of data/command flow
 ``` mermaid
	 sequenceDiagram
	 Roomba/ESP ->> NodeJS: Establish websocket connection
	 Roomba/ESP ->> NodeJS: sends sensor data via websocket
	 NodeJS ->> Web: relays sensor data via websocket
	 Web ->> NodeJS: Send drive command
	 NodeJS ->> Roomba/ESP: relays drive command via websocket	 
```

#### Example of waking roomba up.
Even though the Roomba will go to sleep after 5 minutes of inactivity, the ESP controller is still connected to the NodeJS server via websocket.  This gives us a nice way to wake the Roomba up remotely.
 
 ``` mermaid
	 sequenceDiagram
	  Web ->> NodeJS: Send HTTP Wakeup command
	  NodeJS ->> ESP: Relays websocket Wakeup command to the ESP.
	  ESP ->> Roomba: ESP received the command and triggers I/O pins to wake Roomba
	  ESP ->> NodeJS: Send Roomba sensor data
	  NodeJS ->> Web: Web dashboard comes alive 
```

#### ESP-8266 to Roomba Schematic
[Picture]
 
### Coding goals
A very import criteria of this project is to not make it so overwhelming that it is not fun for my kids.  Therefore, I embarked on some extra effort of wrap a lot of the Roomba and Websocket code into a re-usable classes.  Ok, I am very rusty on C++ and have used it only on basic ESP/Arduino coding so it took me a while to get my sea legs back on that.  I think I did accomplish that well and my teens can just spend time with working on the logical programming of the Roomba.  You can see a few references that I have listed below that have been instrumental to getting this project off the ground. 

### Github
I am very pleased that my son spent 2 solid days to build up the web dashboard using React and it has been working beautifully.  We did most of our work debugging our work using a RaspberryPi to host the NodeJS and React servers and then when we are all happy, we deployed them to Heroku and Vercel and grandparents can see and play with the Roomba.  Main goal accomplished!

### Stretch Goals
1.	Here are some extra credit work that I am working with my daughter on.  Adding a digital compass so that we can try to see if we can build some kind of basic mapping capability to the Roomba.
2.	Add a camera so that we can give the user a POV live feed.

### References
