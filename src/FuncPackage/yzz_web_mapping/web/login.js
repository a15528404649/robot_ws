const form = document.querySelector('#login-form');
const message = document.querySelector('#login-message');
form.addEventListener('submit', async (event) => {
  event.preventDefault(); message.textContent = '正在登录…';
  try {
    const response = await fetch('/api/auth/login', {method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify({username: document.querySelector('#username').value, password: document.querySelector('#password').value})});
    const data = await response.json();
    if (!response.ok) throw new Error(data.error || '登录失败');
    window.location.assign('/');
  } catch (error) { message.textContent = error.message; message.style.color = '#fca5a5'; }
});
