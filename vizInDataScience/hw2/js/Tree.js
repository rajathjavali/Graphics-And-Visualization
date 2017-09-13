/** Class representing a Tree. */
class Tree {
	/**
	 * Creates a Tree Object
	 * parentNode, children, parentName,level,position
	 * @param {json[]} json - array of json object with name and parent fields
	 */
	constructor(json) {
		let i = 0;
		this.treeNodeArray = new Object();
		this.pos = new Array();
		this.treeNodeArray["root"] = new Node("root", undefined);
		for(let item of json)
		{
			this.treeNodeArray[item.name] = new Node(item.name, item.parent);
		}
		//console.log(this.treeNodeArray);
	}

	/**
	 * Function that builds a tree from a list of nodes with parent refs
	 */
	buildTree() {
		for(let iter in this.treeNodeArray)
		{
			if(iter == "root")
			{
				this.parentNode = this.treeNodeArray[iter];
				continue;
			}
			let index = this.treeNodeArray[iter].parentName;
			this.treeNodeArray[iter].parentNode = this.treeNodeArray[index];
			this.treeNodeArray[index].addChild(this.treeNodeArray[iter]);
		}
		this.assignLevel(this.treeNodeArray["root"].children[0], 0);
		this.assignPosition(this.treeNodeArray["root"].children[0], 0);
		//console.log(this.parentNode);
	//Assign Positions and Levels by making calls to assignPosition() and assignLevel()
	}

	/**
	 * Recursive function that assign positions to each node
	 */
	assignPosition(node, position) {
		if(this.pos[node.level] == undefined)
			this.pos[node.level] = 0;

		this.pos[node.level] = this.max(this.pos[node.level], position);
		node.position = this.pos[node.level];

		for(let childIter of node.children)
			this.assignPosition(childIter, this.pos[node.level]);
				
		this.pos[node.level] += 1;
	}

	/**
	 * Recursive function that assign levels to each node
	 */
	assignLevel(node, level) {
		node.level = level;
		for(let childIter of node.children)
		{
			this.assignLevel(childIter, level + 1);			
		}
	}

	/*drawLine(svg, iter) {

		for(let i = 0; i< iter.children.length; i++)
		{
			if(iter.name == "root")
			continue;
			svg.append("line")
				.attr("x1", iter.level * 150 + 100)
				.attr("y1", iter.position * 100 + 100)
				.attr("x2", iter.children[i].level * 150 + 100)
				.attr("y2", iter.children[i].position * 100 + 100);
		}
		for(let it of iter.children)
		this.drawLine(svg, it);
	}*/

	/**
	 * Function that renders the tree
	 */
	renderTree() {
		//execute = function(){
		let body = d3.select("body");
		body.append("svg").attr("width", 1200).attr("height", 1200);
		let svg = d3.select("svg");
		let dataArray = new Array();
		let i = 0;
		for(let iter in this.treeNodeArray){
			if(iter != "root")
			dataArray[i++] = [this.treeNodeArray[iter].level, this.treeNodeArray[iter].position, this.treeNodeArray[iter].name, this.treeNodeArray[iter].parentNode]; 
		}

		console.log(dataArray);
		//this.drawLine(svg, this.parentNode);

		svg.selectAll("line")
			.data(dataArray)
			.enter().append("line")
			.attr("x1", function(d){ return d[0] * 150 + 100;})
			.attr("y1", function(d){ return d[1] * 100 + 120;})
			.attr("x2", function(d){ if(d[3].name == "root") return d[0] * 150 + 75; return d[3].level * 150 + 100;})
			.attr("y2", function(d){ if(d[3].name == "root") return d[1] * 100 + 100; return d[3].position * 100 + 120;});

		svg.selectAll("g")
			.data(dataArray)
			.enter().append("g").append("circle")
			.attr("cx", function(d){ return d[0] * 150 + 100;})
			.attr("cy", function(d){ return d[1] * 100 + 120;})
			.attr("r", 40)
			.style("fill", "steelblue");

		svg.selectAll("g")
			.data(dataArray)
			.append("text")
			.text(function(d){return d[2];})
			.attr("x", function(d){ return d[0] * 150 + 75;})
			.attr("y", function(d){ return d[1] * 100 + 120;})
			.style("fill", "white");
	}

	max(a, b) {
		return (a<b? b:a);
	}
}
