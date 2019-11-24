<html>
    <head>
        <title>DRVR</title>
        <meta name="viewport" content="width=device-width, initial-scale=1.0">
    </head>
    <body>
        <h1>DRVR</h1>
        <div>
            ${frame}<br>
            ${battery}<br>
            ${position}<br>
        </div>
        <div>
            <form action="/turn" method="post">
                <button name="amount" value="-10">left 10</button>
            </form>
            <form action="/next" method="post">
                <button name="action" value="next">Next</button>
            </form>
            <form action="/turn" method="post">
                <button name="amount" value="10">Right 10</button>
            </form>
        </div>
        <div>
            <form action="/halt" method="post">
                <button name="action" value="halt">Halt</button>
            </form>
        </div>
        <div style="height:300px">
            <img src="/image" style="height:300px;transform:rotate(-90deg);padding-top:300px">
        </div>
    </body>
</html>
