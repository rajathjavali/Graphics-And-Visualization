/** Class implementing the infoPanel view. */
class InfoPanel {
    /**
     * Creates a infoPanel Object
     */
    constructor() {
    }

    /**
     * Update the info panel to show info about the currently selected world cup
     * @param oneWorldCup the currently selected world cup
     */
    updateInfo(oneWorldCup) {

        // ******* TODO: PART III *******

        // Update the text elements in the infoBox to reflect:
        // World Cup Title, host, winner, runner_up, and all participating teams that year

        // Hint: For the list of teams, you can create an list element for each team.
        // Hint: Select the appropriate ids to update the text content.

        //Set Labels
		let title = d3.select("#edition");
		title.text(oneWorldCup.EDITION);
		
		let host = d3.select("#host");
		host.text(oneWorldCup.host);
		
		let winner = d3.select("#winner");
		winner.text(oneWorldCup.winner);

		let silver = d3.select("#silver");
		silver.text(oneWorldCup.runner_up);
		
		console.log(oneWorldCup.teams_names);
		
		let teams = d3.select("#teams").selectAll("li");

		let newEle = teams.data(oneWorldCup.teams_names).enter()
			.append("li")
			.text(function(d){return d;});
			
		newEle.exit().remove();
		
		teams = teams.merge(newEle);			
	}

}