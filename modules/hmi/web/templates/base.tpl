<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <link rel="shortcut icon" href="{{ url_for('static', filename='images/favicon.ico') }}">
  <link rel="stylesheet" href="https://maxcdn.bootstrapcdn.com/bootstrap/3.3.7/css/bootstrap.min.css">
  <script src="https://code.jquery.com/jquery-3.2.1.min.js"></script>
  <style>
    body {
      font-family: PingFangSC-Light;
      font-size: 14px;
      color: #ffffff;
      character: 0px;
      line: 20px (1.4);
    }

    .container-fluid {
      width: 100%;
      min-height: 100vh;
      background: #053159;
      padding: 0;
    }
  </style>

  {% block head %} {% endblock %}
  <title>Apollo HMI</title>
</head>

<body>
  <div class="container-fluid center-block">
    <div class="hmi_navbar center-block">
      <style type="text/css" scoped>
        .hmi_navbar {
          background-color: rgba(0, 0, 0, 0.3);
          overflow: auto;
        }
        .hmi_navbar_logo {
          margin-top: 30px;
          margin-left: 30px;
          margin-bottom: 25px;
        }
      </style>

      <img src="{{ url_for('static', filename='images/logo.png') }}"
          class="hmi_navbar_logo" width="141" height="46"/>
    </div>
    <div class="alert alert-danger">
      <strong>Duang!</strong>
      HMI has been integrated with <a onclick="goto_dreamview();">Dreamview</a>.
      Please have a try and help us improve :)
    </div>

    {% block body %} {% endblock %}
  </div>

  <script src="https://maxcdn.bootstrapcdn.com/bootstrap/3.3.7/js/bootstrap.min.js"></script>
</body>
</html>
