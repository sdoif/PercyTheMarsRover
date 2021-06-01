Why React?

React is fast. Apps made in React can handle complex updates and still feel quick and responsive. 
- -> Uses a virtual DOM, this means that it will only update the elements of the webpage which have updated instead of the entire webpage. This is needed as our rover and command interface is dynamic with certain things updating all at different times to one another. 
- This is significant! Only updating the necessary DOM elements is a large part of what makes React so successful. React accomplishes this thanks to something called the virtual DOM. Before moving on to the end of the lesson, read this article about the Virtual DOM.
https://www.codecademy.com/articles/react-virtual-dom
- In React, for every DOM object, there is a corresponding “virtual DOM object.” A virtual DOM object is a representation of a DOM object, like a lightweight copy.

- A virtual DOM object has the same properties as a real DOM object, but it lacks the real thing’s power to directly change what’s on the screen.

- Manipulating the DOM is slow. Manipulating the virtual DOM is much faster, because nothing gets drawn onscreen. Think of manipulating the virtual DOM as editing a blueprint, as opposed to moving rooms in an actual house.

React is modular. Instead of writing large, dense files of code, you can write many smaller, reusable files. React’s modularity can be a beautiful solution to JavaScript’s maintainability problems. 
- This is quite significant as it allows us to focus on the status data segment of our interface and the command porition seperately. This allows us to work on the two things in parallel and not have them intermix and stuff

React is scalable. Large programs that display a lot of changing data are where React performs best. 
- This is the case for our webapp as our rover will be constantly moving and changing its state which our website components will need to react to in order to do things like updating map cooridnates, getting data about state of charge, etc...

UI stuff:
- Made sure to use percentages as much as possible so that the website is infact dynamic with different window sizes 
- We just a colour scheme inspired by the Martian aesthetic (mars color pallete: https://www.color-hex.com/color-palette/7175)

Status Data:
- We were inspired by a car's dashboard for this portion of the website. We decided to take the components of it applicable to our rover and displayed it in a relvant manner 
- This is seen in the speedometer, odometer (for this we need to store all distances traveled from all missions (either in a given session or just through history) in a database), current trip details
- As well we took a more modern/adjusted take on fuel level as it isnt actually fuel and is battery powered? 
