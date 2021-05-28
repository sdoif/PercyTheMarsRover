console.log('Hello World');
console.error('This is an error');
console.warn('This is a warning');
// var, let, const
// var is globally scoped -> dont use
// let and const are locally scoped
// let allows for reassignment 
// const cant be changed or reassigned 
// when to use which? always use const unless you know you will reassign a value
// e.g. a score variable should be let
let score = 30;
console.log(score)
score = score +1;
console.log(score)
// const must be initialized 
const name = 'Salman';
const name2 = "Kai";
const age = 19;
const weight = 45.3;
const x = null; 
const y = undefined;
let z; //undefined atm 
console.log(typeof weight)
//concatenation 
console.log('My name is '+ name +' and I am '+ age)
console.log(`My name is ${name} and I am ${age}`)
const s = "Hello world!";
console.log(s.length); //length is a property, not a method
console.log(s.toUpperCase());
console.log(s.toLowerCase().substr(2,8));
const s2 = "numbers, strings, booleans, nulls"
//to split into an array of elements
console.log(s2.split(', ')); // '' contains the delimiter 
//Arrays
const fruits = ['apples','oranges','pears', 10, true]
console.log(fruits)
//can see that length or type doesnt need to be satisfied or strictly typed
//same goes for variables 
console.log(fruits[fruits.length-1]);
fruits.push('mangoes')
fruits.unshift() //first element removal
console.log(fruits)
fruits.pop() //last element removal
fruits[1] = 'banana'
console.log(fruits)
console.log(Array.isArray(fruits[1]))
console.log(fruits.indexOf(true))
const person = {
    firstName: "Salman",
    lastName: 'Dhaif',
    fruits: ["oranges","apples"]

}
console.log(person.fruits[0])
//destructuring 
const { firstName, lastName } = person;
console.log(firstName)
//adding to dictionary 
person.email = "myemail"
console.log(person)
//JSON -> data format, used in APIs especially when sending data to server
const personJSON = JSON.stringify(person)
console.log(personJSON)

//Control flow
for(let i = 0; i<person.fruits.length; i++){
    console.log(person.fruits[i])
}

//{iterator} of {array}
for(let i of person.fruits){
    console.log(i)
}

//high order array methods: forEach (loops), map (create new array from array), filter (new array based on cond)
//high order methods take in a function as a parameter known as callback function 
//callback func can take in multiple params
//first param is iterator for each item
//array.forEach(function())
person.fruits[2]="bananas"
person.fruits.forEach(function(i){
    console.log(i)
})
//following can be useful for extracting an array from a dictionary
//or if you have an array of dictionaries and you wanna get the values of a specific field for each index in the array
const myFruits = person.fruits.map(function(i){
    return i
})
console.log(myFruits)
person.fruits[3]="berries"

const bFruits = person.fruits.filter(function(i){
    return i[0]==='b';
})
console.log(bFruits)

//chaining high order methods 
const bEndFruits = person.fruits.filter(function(i){
    return i[0]==='b';
}).map(function(i){
    return i.slice(1);
})
console.log(bEndFruits)

//conditionals
const g = '10';
if(g==10){
    console.log("X is: ", g)
}
// == doesnt need to match the data types (maybe it does implicit casting/conversion)
// === the data type NEEDS to match as well as the value
//for the most part you will wanna match the type 
if(g===10){
    console.log("X is: ", g)
}else{
    console.log("X is not a number type with value", 10)
 
}//ternary can also be used to assing variables 
let desc = score>10 ? 'winner' : 'loser';
console.log(desc)

//arrow function
const addNums = () => 1000
console.log(addNums())

//callbacks
const posts =[
    {title: "Post 1", body: "This is post 1" },
    {title: "Post 2", body: "This is post 2" }
];

function getPosts(){
    setTimeout(()=>{
        let output= '';
        posts.forEach((post, index)=>{
            output += `<li>${post.title}</li>`;
        })
        document.body.innerHTML = output;
    },1000);
}
getPosts();

function createPost(post){
    return new Promise((resolve, reject)=>{
    setTimeout(()=>{
        posts.push(post);
        const error = true;

        if(!error){
            resolve();
        }else{
            reject("Error: something went wrong"); //gives uncaught error, should add a catch error
        }
    },2000);
  });
}

createPost({title:"Post 3", body:"This is post 3"})
    .then(getPosts)
    .catch(err=>console.log(err));

//getPosts();
//promises and async data
