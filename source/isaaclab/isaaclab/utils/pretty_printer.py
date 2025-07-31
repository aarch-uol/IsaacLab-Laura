import torch

def print_table(headers, data, title=None, column_alignments=None):

    if not headers or not data:
        print("No data or headers to display.")
        return

    num_columns = len(headers)

    # Determine default column alignments if not provided
    if column_alignments is None:
        column_alignments = ['<' for _ in range(num_columns)] # Default to left-align

    if len(column_alignments) != num_columns:
        raise ValueError("Number of column alignments must match the number of headers.")

    # Calculate maximum width for each column
    column_widths = [len(header) for header in headers] # Start with header widths

   
    for i,cell in enumerate(data):
        column_widths[i] = max(column_widths[i], len(str(cell)))

    # Add padding to column widths
    column_widths = [width + 2 for width in column_widths] # +2 for a bit of padding

    total_table_width = sum(column_widths) + (num_columns + 1) # Sum of widths + vertical bars

    # --- Print the table ---

    # Top border
    print(f"+{'-' * (total_table_width - 2)}+")

    # Title row (if provided)
    if title:
        print(f"| {title.center(total_table_width - 4)} |")
        print(f"+{'-' * (total_table_width - 2)}+") # Separator after title

    # Header separator
    header_separator = "+"
    for width in column_widths:
        header_separator += '-' * width + "+"
    print(header_separator)

    # Header row
    header_row_str = "|"
    for i, header in enumerate(headers):
        alignment_char = column_alignments[i]
        header_row_str += f" {header:{alignment_char}{column_widths[i]-2}} |"
    print(header_row_str)

    # Data separator
    print(header_separator)

    # Data rows
   
    row_str = "|"
    for i, cell in enumerate(data):
        alignment_char = column_alignments[i]
        # Convert cell to string to handle various types
        row_str += f" {str(cell):{alignment_char}{column_widths[i]-2}} |"
    print(row_str)

    # Bottom border
    print(header_separator)

class LogRollout:
    def __init__(self, name, headings):
        self.filename = "docs/logs/" + name + ".txt"
        self.headings = headings
        self._setup()
    
    def _setup(self):
        with open(self.filename, "a") as f:
            for heading in self.headings:
                f.write(heading)
                f.write(' : ')
            f.write(',\n')
    
    def write_to_log(self,data):
        with open(self.filename, "a") as f:
            for log in data:
                if torch.is_tensor(log):
                    log = log.item()
                f.write(str(log))
                f.write(' : ')
            f.write(',\n')

    def new_trial(self, trial):
        with open(self.filename, "a") as f:
            f.write("Trial : ")
            f.write(str(trial))
            f.write(',\n')
    
    def trial_outcome(self, result):
        with open(self.filename, "a") as f:
            f.write("Result : ")
            f.write(str(result))
            f.write(',\n')
    
