Why React?

React is fast. Apps made in React can handle complex updates and still feel quick and responsive. 
- -> Uses a virtual DOM, this means that it will only update the elements of the webpage which have updated instead of the entire webpage. This is needed as our rover and command interface is dynamic with certain things updating all at different times to one another. 

React is modular. Instead of writing large, dense files of code, you can write many smaller, reusable files. React’s modularity can be a beautiful solution to JavaScript’s maintainability problems. 
- This is quite significant as it allows us to focus on the status data segment of our interface and the command porition seperately. This allows us to work on the two things in parallel and not have them intermix and stuff

React is scalable. Large programs that display a lot of changing data are where React performs best. 
- This is the case for our webapp as our rover will be constantly moving and changing its state which our website components will need to react to in order to do things like updating map cooridnates, getting data about state of charge, etc...