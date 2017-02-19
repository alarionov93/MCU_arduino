
/* TODO: create choosing data source dialogs */
$(document).ready(function () {

  $("select#measure").change(function() {
      getData();
  });

  var jsonData;
  function getData() {

      $.ajax({
        method: 'GET',
        url: '/get_measures/' + $("select#measure").val(),
        dataType: 'json',
        contentType:"application/json",
        success: function(resp) {
          jsonData = resp;
          // Load the Visualization API and the corechart package.
          google.charts.load('current', {'packages':['corechart']});
          // Set a callback to run when the Google Visualization API is loaded.
          google.charts.setOnLoadCallback(drawChart);
        }
      });
  }
  getData();

  // Callback that creates and populates a data table,
  // instantiates the pie chart, passes in the data and
  // draws it.
  function drawChart() {
    // Create the data table.
    // var data = new google.visualization.DataTable();
    // data.addColumn('string', 'Abc');
    // data.addColumn('number', 'Temp2');
    // data.addRows(jsonData);
    var data = google.visualization.arrayToDataTable(jsonData);

    // Set chart options
    var options = {'title':'Moto Data Visulizer',
                   'width':1400,
                   'height':800};

    // Instantiate and draw our chart, passing in some options.
    var chart = new google.visualization.LineChart(document.getElementById('chart_div'));
    chart.draw(data, options);
  }
});