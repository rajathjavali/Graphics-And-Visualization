/** Class implementing the table. */
class Table {
    /**
     * Creates a Table Object
     */
    constructor(teamData, treeObject) {

        //Maintain reference to the tree Object; 
        this.tree = treeObject; 

        // Create list of all elements that will populate the table
        // Initially, the tableElements will be identical to the teamData
        this.tableElements = null; // 

        ///** Store all match data for the 2014 Fifa cup */
        this.teamData = teamData;

        //Default values for the Table Headers
        this.tableHeaders = ["Delta Goals", "Result", "Wins", "Losses", "TotalGames"];

        /** To be used when sizing the svgs in the table cells.*/
        this.cell = {
            "width": 70,
            "height": 20,
            "buffer": 15
        };

        this.bar = {
            "height": 20
        };

        /** Set variables for commonly accessed data columns*/
        this.goalsMadeHeader = 'Goals Made';
        this.goalsConcededHeader = 'Goals Conceded';

        /** Setup the scales*/
        this.goalScale = null; 

        /** Used for games/wins/losses*/
        this.gameScale = null; 

        /**Color scales*/
        /**For aggregate columns  Use colors '#ece2f0', '#016450' for the range.*/
        this.aggregateColorScale = null; 

        /**For goal Column. Use colors '#cb181d', '#034e7b'  for the range.*/
        this.goalColorScale = null; 
    }
	
	max(a, b){
		return a>b?a:b;
	}
	
	min(a, b){
		return a<b?a:b;
	}

	/**
     * Creates a table skeleton including headers that when clicked allow you to sort the table by the chosen attribute.
     * Also calculates aggregate values of goals, wins, losses and total games as a function of country.
     *
     */
    createTable() {
		
		let max = this.max;
		let min = this.min;
        
		// ******* TODO: PART II *******
        // Update Scale Domains
				
		let dataMax = d3.max(this.teamData, d => {return max(d.value[this.goalsMadeHeader], d.value[this.goalsConcededHeader])});
		//let dataMin = d3.min(data, d => {return min(d.value[goalsMade], d.value[goalsConceded])});
		
		this.goalScale = d3.scaleLinear()
						.domain([0, dataMax])
						.range([this.cell.buffer, 2.5 * this.cell.width - this.cell.buffer]);
        
		
		// Create the x axes for the goalScale.
		let goalAxis = d3.axisBottom();
		goalAxis.scale(this.goalScale);
		
        //add GoalAxis to header of col 1.
		let table = d3.select("#goalHeader")
						.append("svg")
						.attr("width", 2.5 * this.cell.width)
						.attr("height", this.cell.height);
		table.append("g")
				.call(goalAxis);

				
		this.tableElements = this.teamData.slice();
		
		// ******* TODO: PART V *******

        // Set sorting callback for clicking on headers

        // Clicking on headers should also trigger collapseList() and updateTable(). 

       
    }


    /**
     * Updates the table contents with a row for each element in the global variable tableElements.
     */
    updateTable() {
        
		//---- min and max functions ---------
		let max = this.max, min = this.min;
		
		//----------- Goal Scale --------------------
		let goalScale = this.goalScale;
		
		//------------- cell center -----------
		let cellcenter = this.cell.height/2;
		
		//------------------------ Color and Bar Scale ----------------------------------
		let dataMax, dataMin, maxgames, colorScale, barScale;
		
		dataMax = d3.max(this.tableElements, d => d.value[this["Delta Goals"]]);
		dataMin = d3.min(this.tableElements, d => d.value[this["Delta Goals"]]);
		maxgames = d3.max(this.tableElements, d=> d.value.TotalGames);
		
		colorScale = d3.scaleLinear()
							.domain([0, maxgames])
							.range(["#e6ffe6", "#003300"]);
		
		barScale = d3.scaleLinear()
							.domain([0, maxgames])
							.range([0, this.cell.width - this.cell.buffer]);
		
		// ******* TODO: PART III *******
        //--------------------------------- Create table rows --------------------------------------------
		//---------------------------- Updating table and elements ---------------------------------------
		console.log(this.tableElements);
		let table = d3.select("#matchTable").select("tbody").selectAll("tr").data(this.tableElements);
		
		let tr = table.enter()
						.append("tr")
						.on("click", (d, i)=>this.updateList(i));
		
		table.exit().remove();
		table = tr.merge(table);
		
		table.attr("id", d=>d.key)
			 .attr("class", d=>d.value.type);


        //Append th elements for the Team Names
		let th = table.selectAll("th").data(d=>[d.key]);
		
		let newth =	th.enter()
					.append("th")
					.attr("width", this.cell.width)
					.attr("height", this.cell.height);;
					
		th = newth.merge(th);
		th.text(d=>d);
		

        //Append td elements for the remaining columns.
		//Data for each cell is of the type: {'type':<'game' or 'aggregate'>, 'value':<[array of 1 or two elements]>}
		let td = table.selectAll("td")
					.data(d=>[
								{type: d.value.type, vis: "goals",  value:[{type: d.value.type, delta: d.value[this.goalsMadeHeader] - d.value[this.goalsConcededHeader], goals: d.value[this.goalsMadeHeader]}, {type: d.value.type, delta: d.value[this.goalsMadeHeader] - d.value[this.goalsConcededHeader], goals: d.value[this.goalsConcededHeader]}]},
								{type: d.value.type, vis: "text",  value:[d.value.Result.label]},
								{type: d.value.type, vis: "bar",  value:[d.value.Wins]},
								{type: d.value.type, vis: "bar",  value:[d.value.Losses]},
								{type: d.value.type, vis: "bar",  value:[d.value.TotalGames]}
							]);
		let newtd = td.enter().append("td");
		
		td.exit().remove();
		td = newtd.merge(td);

		let svg = td.filter(function(d){return d.vis == "goals";});
					//.append("svg").attr("width", this.cell.width * 2).attr("height", this.cell.height);

		let newsvg = svg.selectAll("svg").data(function(d){return d3.select(this).data();}).enter()
						.append("svg")
						.attr("width", this.cell.width * 2.5)
						.attr("height", this.cell.height);

		svg.exit().remove();
		svg = newsvg.merge(svg);
		
		let goalBars = svg.selectAll("rect").data(function(d){return d3.select(this).data();})
		let goalbar = goalBars.enter().append("rect");
		
		goalBars = goalbar.merge(goalBars);
		
		goalBars.attr("x", function(d){
				let minVal = min(d.value[0].goals, d.value[1].goals);
				return goalScale(minVal);
			})
			.attr("y", function(d){
				let val = -5;
				if(d.type == "game")
						val = -2.5
				return cellcenter + val;
			})
			.attr("height", function(d){if(d.type == "game") return 5; return 10;})
			.attr("width", function(d){
				let minVal = min(d.value[0].goals, d.value[1].goals);
				let maxVal = max(d.value[0].goals, d.value[1].goals);
				return goalScale(maxVal) - goalScale(minVal);
			})
			.attr("style", function(d){
				if(d.value[0].delta < 0)
					return "fill: #f3988c";
				else
					return "fill: #a8bad6";
			})
			.on("mouseover", function(d){
				d3.select(this).append("title").text("Goals Scored: "+d.value[0].goals+"\nGoals Conceded: "+d.value[1].goals);
			})
			.on("mouseout", function(d){
				d3.select(this).select("title").remove();
			});
		
		
		let circles = svg.selectAll("circle").data(d=>d.value)
		let circle = circles.enter().append("circle");
		
		circles.exit().remove();
		circles = circle.merge(circles);
		
		circles.attr("cx", d=>goalScale(d.goals))
			.attr("cy", cellcenter)
			.attr("r", 5)
			.attr("style", function(d,i){
				let val;
				if(d.type == "game")
				{
					if(i == 0)
						val = "stroke: #364e74; stroke-width: 2px; fill: white";
					else
						val = "stroke: #be2714; stroke-width: 2px; fill: white";
					if(d.delta == 0)
						val = "stroke: grey; stroke-width: 2px; fill: white";
				}
				else
				{
					if(i == 0)
						val = "fill: #364e74";
					else
						val = "fill: #be2714";
					if(d.delta == 0)
						val = "fill: grey";
				}
				return val;
			})
			.on("mouseover", function(d){
				d3.select(this)
					.append("title")
					.text(d); 
			})
			.on("mouseoout", function(d){
				d3.select(this).remove("title");
			});
        //------------------------------------- Round/Results ------------------------------------------------
        let results = td.filter(function(d){return d.vis == "text";})
						.selectAll("svg")
						.data(function(d){return d3.select(this).data();})
		let resultsvg = results.enter().append("svg")
							.attr("width", 2.5 * this.cell.width)
							.attr("height", this.cell.height)
							.selectAll("text")
							.data(d=>d.value)
							.enter()
							.append("text")
							.attr("x", 0)
							.attr("y", cellcenter);
		results.exit().remove();
		results = resultsvg.merge(results);
		
		results
				.text(d=>d);
		
		//------------------------------------- Wins, Losses, TotalGames -------------------------------------
		let barsSvg = td.filter(function(d){return d.vis == "bar";});
		let newBarSvg = barsSvg.selectAll("svg").data(function(d){return d3.select(this).data();}).enter()
						.append("svg")
						.attr("width", this.cell.width)
						.attr("height", this.cell.height);
		
		barsSvg.exit().remove();
		barsSvg = newBarSvg.merge(barsSvg);
						
		let bars = barsSvg.selectAll("rect").data(function(d){return d3.select(this).data();});
		let newBar = bars.enter().append("rect");
		
		bars.exit().remove();
		bars = newBar.merge(bars);
		
		bars.attr("x", 0)
			.attr("y", 0)
			.attr("height", this.cell.height)
			.attr("width", function(d){return barScale(d.value[0]);})
			.attr("style", d=>"fill:"+colorScale(d.value[0]))
		
		let textf = barsSvg.selectAll("text").data(function(d){return d3.select(this).data();});
		let newText = textf.enter().append("text");
	
		textf.exit().remove();
		textf = newText.merge(textf);
				
		textf.text(d=>d.value[0])
			.attr("x", d=>barScale(d.value[0])-2)
			.attr("y", this.cell.height/4 * 3)
			.attr("style", "font-size: 12px ;fill: white; text-anchor: end");
		
        //Add scores as title property to appear on hover

        //Populate cells (do one type of cell at a time )

        //Create diagrams in the goals column

        //Set the color of all games that tied to light gray

    };

    /**
     * Updates the global tableElements variable, with a row for each row to be rendered in the table.
     *
     */
    updateList(i) {
        // ******* TODO: PART IV *******
       
        //Only update list for aggregate clicks, not game clicks
		let newlist, newlen = 0;
		let len = this.tableElements.length;
        if(this.tableElements[i].value.type == "aggregate" && (i == (len-1) ? true : this.tableElements[i+1].value.type == "aggregate"))
		{
			newlist = this.tableElements.splice(0, i+1);
			newlist = (newlist.concat(newlist[i].value.games.slice())).slice();
			for(let k = 1; k <= newlist[i].value.games.length;k++){
				let key = "x" + newlist[i].value.games[k-1].key;
				newlist[i+k].key = key;
			}
			console.log(newlist);
			this.tableElements = newlist.concat(this.tableElements);
		}
		else if (this.tableElements[i].value.type == "aggregate" && (i == (len-1) ? true : this.tableElements[i+1].value.type == "game"))
		{
			newlen = this.tableElements[i].value.games.length;
			newlist = this.tableElements.splice(i+1, newlen);
			for(let k = 0; k < this.tableElements[i].value.games.length; k++)
				this.tableElements[i].value.games[k].key =
					this.tableElements[i].value.games[k].key.replace("x","");
		}
		else
			return;
		
		this.updateTable();
		
    }

    /**
     * Collapses all expanded countries, leaving only rows for aggregate values per country.
     *
     */
    collapseList() {
		let count = 0;
		for(let i = this.tableElements.length-1; i>=0; i--)
		{
			
			if(this.tableElements[i].value.type == "game")
			{
				count++;
			}
			else
			{
				if(count != 0)
				{
					this.tableElements.splice(i+1, count);
				}
				count = 0;
			}
		}
        // ******* TODO: PART IV *******
		this.updateTable();
    }


}
