<html>
    <head>
        <title>DRVR</title>
        <meta name="viewport" content="width=device-width, initial-scale=1.0">
    </head>
    <body>
        <h1>DRVR</h1>
        <div>
            ${directory}<br>
            ${frame}<br>
            ${battery}<br>
            ${position}<br>
            ${predictions}<br>
        </div>
        <div>
            <form action="/turn" method="post">
                <button name="amount" value="-5">left 5</button>
            </form>
            <form action="/next" method="post">
                <button name="action" value="next">Next</button>
            </form>
            <form action="/turn" method="post">
                <button name="amount" value="5">Right 5</button>
            </form>
            <form action="/correct" method="post">
                <button name="direction" value="left">Correct Left</button>
            </form>
            <form action="/correct" method="post">
                <button name="direction" value="right">Correct Right</button>
            </form>
            <form action="/act" method="post">
                <button name="action" value="act">Action</button>
            </form>
        </div>
        <div>
            <form action="/halt" method="post">
                <button name="action" value="halt">Halt</button>
            </form>
        </div>
        <div style="height:300px">
            <img src="/image" style="height:300px;transform:rotate(180deg)">
        </div>
    </body>
</html>
