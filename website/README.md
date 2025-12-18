# Modular Drone Control System - Documentation Website

A professional, clean documentation website for the ROS2 Modular Drone Control System project.

## Features

- **Responsive Design**: Works seamlessly on desktop, tablet, and mobile devices
- **Dark Mode Ready**: Built-in dark mode support with Tailwind CSS
- **Professional Styling**: Clean, modern design inspired by technical documentation standards
- **Interactive Navigation**: Smooth scrolling, active section highlighting, and sticky sidebar
- **Video Demo Section**: Three side-by-side video placeholders for demonstrations

## File Structure

```
website/
├── index.html          # Main HTML file
├── css/
│   └── style.css       # Custom CSS styles
├── js/
│   └── main.js         # JavaScript for interactivity
├── assets/             # Directory for images and videos
└── README.md           # This file
```

## Adding Demo Videos

To add your demo videos to the website:

1. **Prepare your videos** in MP4 format (recommended for best browser compatibility)
   - ArUco marker tracking demo
   - Hand gesture control demo
   - Follow me mode demo

2. **Place video files** in the `website/assets/` directory:
   ```bash
   cp aruco-demo.mp4 website/assets/
   cp gesture-demo.mp4 website/assets/
   cp follow-demo.mp4 website/assets/
   ```

3. **Update index.html** by uncommenting the video tags in the "Demo Videos" section:

   Find these lines (around line 450-490):
   ```html
   <!-- Replace with actual video -->
   <!-- <video class="w-full h-full object-cover" controls>
       <source src="assets/aruco-demo.mp4" type="video/mp4">
   </video> -->
   ```

   Uncomment them to:
   ```html
   <video class="w-full h-full object-cover" controls>
       <source src="assets/aruco-demo.mp4" type="video/mp4">
   </video>
   ```

4. Do the same for all three video sections (ArUco, Gesture, Follow)

## Viewing the Website

### Option 1: Direct File Opening
Simply open `index.html` in your web browser:
```bash
open website/index.html
```

### Option 2: Local Web Server (Recommended)
For best results, serve the website with a local web server:

**Using Python:**
```bash
cd website
python3 -m http.server 8000
```
Then visit: http://localhost:8000

**Using Node.js (with npx):**
```bash
cd website
npx http-server -p 8000
```
Then visit: http://localhost:8000

**Using PHP:**
```bash
cd website
php -S localhost:8000
```
Then visit: http://localhost:8000

## Customization

### Updating Content

1. **Team Information**: Edit the team section in `index.html` (around line 620)
   - Add/remove team member cards
   - Update names, roles, and initials

2. **GitHub Link**: Update the GitHub repository URL:
   - Header navigation (line 38)
   - Footer (line 657)

3. **Project Details**: Modify section content in `index.html` to match your specific implementation

### Styling

- **Colors**: Update the Tailwind config in `index.html` (lines 9-34)
- **Custom styles**: Edit `css/style.css`
- **Animations**: Modify JavaScript in `js/main.js`

### Adding Images

Place images in the `assets/` directory and reference them:
```html
<img src="assets/your-image.jpg" alt="Description" class="..." />
```

## Browser Compatibility

The website is compatible with:
- Chrome/Edge (recommended)
- Firefox
- Safari
- Opera

## Technologies Used

- **HTML5**: Semantic markup
- **Tailwind CSS**: Utility-first CSS framework (via CDN)
- **Vanilla JavaScript**: No framework dependencies
- **Google Fonts**: Inter font family
- **Material Symbols**: Icon font

## Features Highlights

### Navigation
- Fixed header with navigation links
- Sticky sidebar with table of contents
- Active section highlighting on scroll
- Smooth scroll behavior

### Sections
1. **Introduction**: Project overview with feature cards
2. **System Architecture**: Component breakdown
3. **ArUco Marker Mode**: Technical details with code snippets
4. **Hand Gesture Control**: Gesture recognition explanation
5. **Follow Me Mode**: Autonomous tracking details
6. **Demo Videos**: Three side-by-side video showcases
7. **Conclusion**: Future enhancements
8. **Team**: Team member profiles

### Accessibility
- Semantic HTML structure
- ARIA labels where appropriate
- Keyboard navigation support
- High contrast color schemes

## Deployment

### GitHub Pages
1. Push the website to your repository
2. Go to Settings → Pages
3. Select branch and `/website` folder
4. Your site will be available at: `https://yourusername.github.io/repo-name/`

### Netlify
1. Drag and drop the `website/` folder to Netlify
2. Or connect your GitHub repository
3. Set publish directory to `website/`

### Vercel
```bash
cd website
vercel
```

## License

Part of the Modular Drone Control System project.

## Support

For issues or questions about the website, please open an issue in the GitHub repository.
