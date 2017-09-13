# CS-5630 / CS-6630 Homework 2
*Due: Friday, September 8, 11:59 pm.*

In this assignment you will use JavaScript to create a tree data structure, and then render that tree to the browser with the D3 library.

As in Homework 1, add your name, your e-mail address, and your UID to the HTML elements at the top. Also make sure your submission is a valid HTML5 file. Check that it is valid by uploading it to the [W3C HTML Validator](https://validator.w3.org/#validate_by_upload).

Your project structure looks like this:

	hw2/
		README.md
		hw2.html
		js/
			Node.js
			Tree.js
			script.js
		data/
		  Tree.json

Remember, to be able to access the data files with JavaScript, you will need to be *serving* the hw2 directory, not just opening the HTML file in a browser. If your development environment doesn't already launch a server for you, you can start one with one of these commands:

	$ cd path/to/hw2
	# for python 2
	$ python -m SimpleHTTPServer 8080
	# for python 3
	$ python -m http.server 8080

You can view the page at [http://localhost:8080](http://localhost:8080).

In this assignment we will be using JavaScript classes, as covered in lecture. The homework consists of two parts:

1. Manipulating data with JavaScript, and
2. Rendering elements to the browser using basic D3 functionality.

All your work for this HW will be in the `js/Tree.js` file. All other necessary code has been provided for you. Your final tree should look similar to this one:

![caption goes here](figs/finalTree.png)


## The Data and the Data Structure

The Node class has been provided in the `js/node.js` file and will be used as the data structure of the nodes in the tree.

The source data from which to construct the tree lives in the `data/Tree.json` file (see snippet below). This JSON file contains a list of objects, where each object has a name, and references the name of its parent.

```JSON
[{
"name":"Animal",
"parent":"root"
},
{
"name":"Reptile",
"parent":"Animal"		
},
{
"name":"Mammal",
"parent":"Animal"		
}, ...
```

Notice that the constructor of this class takes in two parameters: the name of the node as a string, and the name of the parent, also as a string.

```JS
constructor(nodeName, parentName) {

	//String of Node Name
	this.name = nodeName;

	//String of Parent Name
	this.parentName = parentName;

	/**
	 * Reference to parent Node Object.
	 */
	this.parentNode = null;

	/**
	 * Array of Children.
	 */
	this.children=[];

	/**
	 * Level of the node.
	 */
	this.level=null;

	/**
	 * Position of the node.
	 * Initialize to -1
	 */
	this.position=-1;
}

```

You will call this constructor from the Tree class for every node in the JSON to create an array of node objects. The constructor also initializes the remaining attributes of the Node class. Notice that there are both `level` and `position` attributes. The level refers to how deep in the tree a given node is (where the root is level 0), while the position refers to where a given node is in relation to other nodes in the same level. These two attributes will be important when rendering the tree to the browser and are discussed in more detail later in this spec.

The Tree class will have two main functions:

1. Data wrangling, where we will create a tree data structure from the list of node objects, and
2. Rendering the tree to the screen.

We'll start with the data wrangling.


## Part I: Data Wrangling

We have provided the skeleton of the tree class in `js/Tree.js`. Your job is to fill out the constructor and functions below.

Our tree class will only need to have one attribute: a list (represented by an array) of objects of the class `Node`.

Let's start by filling out the constructor of the Tree class, which takes in an array of JSON objects.

```JS
/**
 * Creates a Tree Object
 * parentNode, children, parentName, level, position
 * @param {json[]} json - array of JSON object with name and parent fields
 */
constructor(json) {

}
```

Add code that will create this list of `Node` objects based on the input. This is a good place to populate the `parentNode` field of the `Node` objects as well.

The next step is to complete the `buildTree()` function.

```JS
/**
 * Function that builds a tree from a list of nodes with parent refs
 */
buildTree() {

}

```

This function should use the newly created list of `Node` objects and add a link to the node's children. This function should also call the `assignLevel()` and `assignPosition()` functions, that we will implement in the next step. Your Node objects should look like this:

![Structure of the node](figs/nodeStructure.png)

Once all nodes have the correct parent and child references, we can proceed to the two recursive functions, `assignLevel` and `assignPosition`. Note that it is possible to do both, assigning levels and assigning positions in one sweep - feel free to do that if you prefer.

The level of a node is how 'deep' it is in the tree. The level of the root is 0, the children of the root are 1, the children of the children are 2, and so on. The position of a node is at least the position of its parent, and for every extra child of a parent the position increases. This lets us create a simple tree layout, without using any of D3's layout functions.

The figure below shows the final tree in this homework, with labels on each node in the form **Level , Position**.

![Tree with levels](figs/levelPos.png)

Both of these functions are best implemented recursively, so that you can recurse through the tree and assigning the level and position values as you go. We recommend you implement the `assignLevel` function first since you may need the `level` attribute of a node in helping to determine its position.


## Part II: Rendering the Tree

Now that we have the data in the correct structure, we are ready to render this tree to the screen.  

In order to do this, we will implement our last function, `renderTree()`. As the name implies, this function will render all the nodes in your instance of the `Tree` class to the screen. Note that we will not be using D3's build in tree layout functions - we've already calculated the necessary information for our layout in the first section.

The first step is to append an SVG element to the body of your page. We recommend sizing the SVG to something around 1200 x 1200 to give plenty of room for your nodes, edges, and labels.

Once you have an SVG, you will use:

* circles to represent the nodes,
* line elements for the edges, and
* text elements for the labels.

Notice that we have provided the css rules in `hw2.html` for circle and line elements, as well as a class called label that you can apply to your text elements.

### Scaling and Positioning

When positioning your SVG elements, use the `level` attribute of the node to calculate the x value of the circle and the position attribute to calculate the y value. Note that you need to scale the level and position values, in order to get reasonable values for the x and y attributes. 

A few aesthetic things to keep in mind:

1. Make the Tree big enough so that we can see it properly.
2. Render the edges connecting the nodes underneath the nodes.
3. Make sure there are no edge crossings.
4. Ensure all edges end at a position value greater or equal to that where they started (this is a result of the logic implemented in `assignPosition()`).
5. We don't assume the tree to be ordered, so any order of leafs is acceptable. 

Here is an example of an *incorrect* tree, where the nodes of the fourth level are incorrectly positioned.

![Wrong Tree](figs/wrong.png)

Have fun with your first D3 homework and make sure to come to the HW Lab if anything is unclear.

## Grading

Your score on this assignment will be as follows:

10%: The constructor properly creates an array of `Node` objects.   
15%: The `buildTree` function properly populates the children.   
15%: The levels are correctly assigned.  
20%: The positions are correctly assigned.  
40%: The Tree is properly displayed in the browser, using the appropriate D3 syntax in the renderTree function.
