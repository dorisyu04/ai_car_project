function sendCmd(action) {
    fetch('/control', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/x-www-form-urlencoded'
        },
        body: 'action=' + encodeURIComponent(action)
    })
    .then(response => response.text())
    .then(data => {
        console.log('伺服器回應:', data);
        const msgBox = document.getElementById('ai-msg');
        if (msgBox) {
            msgBox.textContent = '目前指令：' + action;
        }
    })
    .catch(error => {
        console.error('錯誤:', error);
    });
}
