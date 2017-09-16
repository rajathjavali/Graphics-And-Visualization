/**
 * Makes the first bar chart appear as a staircase.
 *
 * Note: use only the DOM API, not D3!
 */
function staircase() {
    // ****** TODO: PART II ******
    let barGraph1 = document.getElementById("bar1");
    let barXtoHeight = new Array();
    let i =0;
    for(let iter of barGraph1.children)
        barXtoHeight[i++] = iter.attributes.height.nodeValue;
    
    barXtoHeight.sort(function(a, b) {
        return parseInt(a) - parseInt(b);
    });
    
    //console.log(barXtoHeight);
    
    i = 0;
    for(let iter of barGraph1.children) {
        iter.attributes.height.nodeValue = barXtoHeight[i++];
    }
    //console.log(barGraph1);
}

/**
 * Render the visualizations
 * @param error
 * @param data
 */
function update(error, data) {
    if (error !== null) {
        alert('Could not load the dataset!');
    } else {
        // D3 loads all CSV data as strings;
        // while Javascript is pretty smart
        // about interpreting strings as
        // numbers when you do things like
        // multiplication, it will still
        // treat them as strings where it makes
        // sense (e.g. adding strings will
        // concatenate them, not add the values
        // together, or comparing strings
        // will do string comparison, not
        // numeric comparison).

        // We need to explicitly convert values
        // to numbers so that comparisons work
        // when we call d3.max()

        for (let d of data) {
            d.a = +d.a;
            d.b = +d.b;
        }
    }

    // Set up the scales
    let aScale = d3.scaleLinear()
        .domain([0, d3.max(data, d => d.a)])
        .range([0, 150]);
    let bScale = d3.scaleLinear()
        .domain([0, d3.max(data, d => d.b)])
        .range([0, 150]);
    let iScale = d3.scaleLinear()
        .domain([0, data.length])
        .range([0, 110]);

    // console.log(data);
    // ****** TODO: PART III (you will also edit in PART V) ******
    // TODO: Select and update the 'a' bar chart bars

    //------------------------------------------------------------------------------------------- Bar Graph 1 ---------------------------------------------------------------------------
    let bar1 = d3.select("#bar1");
    let bar1Rect = bar1.selectAll("rect").data(data);

    bar1Rect.enter()
        .append("rect")
        .on("mouseover", function() {d3.select(this).style("fill", "orange")})
        .on("mouseout", function() {d3.select(this).style("fill", "steelblue")})
		.attr("x", (d, i) => iScale(i))
        .attr("y", 0)
        .attr("width", 10)
        .attr("height", 0)
        .style("opacity", 0)
        .transition()
        .duration(2000)
        .style("opacity", 1)
        .attr("height", d => aScale(d.a))
        .style("fill", "steelblue");
        


    bar1Rect.exit()
        .attr("height", d => aScale(d.a))
        .transition()
        .duration(500)
        .attr("height", 0)
        .remove();
    
    bar1Rect = bar1Rect.merge(bar1Rect);

    bar1Rect
        .on("mouseover", function() {d3.select(this).style("fill", "orange")})
        .on("mouseout", function() {d3.select(this).style("fill", "steelblue")})
		.transition()
		.duration(500)
		.attr("y", 0)
        .attr("width", 10)
		.attr("height", 0)//d => aScale(d.a))
        .transition()
        .duration(1000) 
		.attr("x", (d, i) => iScale(i))
        .attr("height", d => aScale(d.a))
        .style("fill", "steelblue");
        


    // TODO: Select and update the 'b' bar chart bars

    //------------------------------------------------------------------------------------------- Bar Graph 2 ---------------------------------------------------------------------------
    let bar2 = d3.select("#bar2");
    let bar2Rect = bar2.selectAll("rect").data(data);
                    
    bar2Rect.enter()
        .append("rect")
		.attr("x", (d, i) => iScale(i))
        .attr("y", 0)
        .attr("width", 10)
        .style("opacity", 0)
        .attr("height", 0)
        .on("mouseover", function() {d3.select(this).style("fill", "red")})
        .on("mouseout", function() {d3.select(this).style("fill", "steelblue")})
        .transition()
        .duration(2000)
        .style("opacity", 1)
        .attr("height", d => bScale(d.b))
        .style("fill", "steelblue");
    
    bar2Rect.exit()
		.attr("height", d => aScale(d.a))
        .transition()
        .duration(500)
        .attr("height", 0)
        .remove();
    
    bar2Rect = bar2Rect.merge(bar2Rect);

    bar2Rect
        .on("mouseover", function() {d3.select(this).style("fill", "red")})
        .on("mouseout", function() {d3.select(this).style("fill", "steelblue")})
		.transition()
		.duration(500)
        .attr("y", 0)
        .attr("width", 10)
        .attr("height", 0)
        .transition()
        .duration(1000)
        .attr("x", (d, i) => iScale(i))
        .attr("height", d => bScale(d.b))
        .style("fill", "steelblue");

    // TODO: Select and update the 'a' line chart path using this line generator

    //------------------------------------------------------------------------------------------- Line Graph 1 ---------------------------------------------------------------------------
    let aLineGenerator = d3.line()
        .x((d, i) => iScale(i))
        .y((d) => aScale(d.a));

    let line1 = d3.select("#line1");
    let lineGraph1 = line1.select("path").data(data)
                        .style("opacity", 0)
                        .transition()
                        .duration(1000)
                        .style("opacity", 1)
                        .attr("d", aLineGenerator(data))
                        .attr("stroke", "steelblue")
                        .attr("stroke-width", 1)
                        .attr("fill", "none");
 
    //------------------------------------------------------------------------------------------- Line Graph 2 ---------------------------------------------------------------------------
    let bLineGenerator = d3.line()
        .x((d, i) => iScale(i))
        .y((d) => aScale(d.b));

    let line2 = d3.select("#line2");
    let lineGraph2 = line2.select("path").data(data)
                        .style("opacity", 0)
                        .transition()
                        .duration(1000)
                        .style("opacity", 1)
                        .attr("d", bLineGenerator(data))
                        .attr("stroke", "steelblue")
                        .attr("stroke-width", 1)
                        .attr("fill", "none");
    // TODO: Select and update the 'b' line chart path (create your own generator)

    // TODO: Select and update the 'a' area chart path using this area generator

    //------------------------------------------------------------------------------------------- Area Graph 1 ---------------------------------------------------------------------------
    let aAreaGenerator = d3.area()
        .x((d, i) => iScale(i))
        .y0(0)
        .y1(d => aScale(d.a));

    let area1 = d3.select("#area1");
    let areaGraph1 = area1.select("path").data(data)
                        .style("opacity", 0)
                        .transition()
                        .duration(1000)
                        .style("opacity", 1)
                        .attr("d", aAreaGenerator(data))
                        .attr("stroke", "steelblue")
                        .attr("stroke-width", 1)
                        .attr("fill", "steelblue");

    //------------------------------------------------------------------------------------------- Area Graph 2 ---------------------------------------------------------------------------
    let bAreaGenerator = d3.area()
        .x((d, i) => iScale(i))
        .y0(0)
        .y1(d => aScale(d.b));

    let area2 = d3.select("#area2");
    let areaGraph2 = area2.select("path").data(data)
                        .style("opacity", 0)
                        .attr("stroke", "steelblue")
                        .attr("stroke-width", 1)
                        .attr("fill", "steelblue")
                        .transition()
                        .duration(1000)
                        .attr("d", bAreaGenerator(data))
                        .style("opacity", 1);
    // TODO: Select and update the 'b' area chart path (create your own generator)

    // TODO: Select and update the scatterplot points

    //------------------------------------------------------------------------------------------- ScatterPlot ----------------------------------------------------------------------------
    let scatterplot = d3.select("#scatterplot");
    let circles = scatterplot.selectAll("circle").data(data);

    circles.enter()
        .append("circle")
        .attr("cx", d => aScale(d.a))
        .attr("cy", d => bScale(d.b))
		.attr("opacity", 0)
		.transition()
		.duration(1500)
		.attr("opacity", 1)
        .attr("r", 5)
        .style("fill", "steelblue");


    circles.exit()
        .attr("opacity", 1)
        .transition()
        .duration(500)
        .attr("opacity", 0)
        .remove();
    
    circles = circles.merge(circles);

    circles
        .transition()
        .duration(1000)
        .attr("cx", d => aScale(d.a))
        .attr("cy", d => bScale(d.b))
        .attr("r", 5)
        .style("fill", "steelblue");
    // ****** TODO: PART IV ******

    circles.on("click", function() {
        let coords = d3.mouse(this);
        let display = "x: "+coords[0]+", y: "+coords[1];
        console.log(display);   
    });

    circles.on("mouseover", function(d) {
        let coords = d3.mouse(this);
        d3.select(this)
        .append("title")
        .text("Mouse Coord:\nx: "+ coords[0]+ " y: "+coords[1] + "\nPoint:\nx: " + d.a + " y: "+ d.b ) 
    });

    /*circles.on("mouseout", function() {
        let coords = d3.mouse(this);
        d3.select(this)
        .select("title")
        .remove()      
    });*/

}

/**
 * Load the file indicated by the select menu
 */
function changeData() {
    let dataFile = document.getElementById('dataset').value;
    if (document.getElementById('random').checked) {
        randomSubset();
    }
    else {
        d3.csv('data/' + dataFile + '.csv', update);
    }
}

/**
 *   Load the file indicated by the select menu, and then slice out a random chunk before passing the data to update()
 */
function randomSubset() {
    let dataFile = document.getElementById('dataset').value;
    if (document.getElementById('random').checked) {
        d3.csv('data/' + dataFile + '.csv', function (error, data) {
            let subset = [];
            for (let d of data) {
                if (Math.random() > 0.5) {
                    subset.push(d);
                }
            }
            update(error, subset);
        });
    }
    else {
        changeData();
    }
}