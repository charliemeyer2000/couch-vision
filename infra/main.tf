terraform {
  required_providers {
    aws = {
      source  = "hashicorp/aws"
      version = "~> 5.0"
    }
  }
}

provider "aws" {
  region = "us-east-1"
}

# --- S3 Bucket ---

resource "aws_s3_bucket" "bags" {
  bucket = "couch-vision-bags"
}

resource "aws_s3_bucket_public_access_block" "bags" {
  bucket = aws_s3_bucket.bags.id

  block_public_acls       = false
  block_public_policy     = false
  ignore_public_acls      = false
  restrict_public_buckets = false
}

resource "aws_s3_bucket_policy" "bags_public_read" {
  bucket     = aws_s3_bucket.bags.id
  depends_on = [aws_s3_bucket_public_access_block.bags]

  policy = jsonencode({
    Version = "2012-10-17"
    Statement = [
      {
        Sid       = "PublicReadGetObject"
        Effect    = "Allow"
        Principal = "*"
        Action    = "s3:GetObject"
        Resource  = "${aws_s3_bucket.bags.arn}/*"
      }
    ]
  })
}

resource "aws_s3_bucket_notification" "bags_notification" {
  bucket = aws_s3_bucket.bags.id

  lambda_function {
    lambda_function_arn = aws_lambda_function.index_generator.arn
    events              = ["s3:ObjectCreated:*", "s3:ObjectRemoved:*"]
  }

  depends_on = [aws_lambda_permission.allow_s3]
}

# --- Lambda: index.txt generator ---

data "aws_iam_policy_document" "lambda_assume" {
  statement {
    actions = ["sts:AssumeRole"]
    principals {
      type        = "Service"
      identifiers = ["lambda.amazonaws.com"]
    }
  }
}

resource "aws_iam_role" "lambda" {
  name               = "couch-vision-index-generator"
  assume_role_policy = data.aws_iam_policy_document.lambda_assume.json
}

resource "aws_iam_role_policy" "lambda" {
  name = "s3-access"
  role = aws_iam_role.lambda.id

  policy = jsonencode({
    Version = "2012-10-17"
    Statement = [
      {
        Effect   = "Allow"
        Action   = ["s3:ListBucket"]
        Resource = aws_s3_bucket.bags.arn
      },
      {
        Effect   = "Allow"
        Action   = ["s3:PutObject"]
        Resource = "${aws_s3_bucket.bags.arn}/index.txt"
      },
      {
        Effect   = "Allow"
        Action   = ["logs:CreateLogGroup", "logs:CreateLogStream", "logs:PutLogEvents"]
        Resource = "arn:aws:logs:*:*:*"
      }
    ]
  })
}

data "archive_file" "index_generator" {
  type        = "zip"
  source_file = "${path.module}/lambda/index_generator.py"
  output_path = "${path.module}/lambda/index_generator.zip"
}

resource "aws_lambda_function" "index_generator" {
  function_name    = "couch-vision-index-generator"
  role             = aws_iam_role.lambda.arn
  handler          = "index_generator.handler"
  runtime          = "python3.12"
  timeout          = 30
  filename         = data.archive_file.index_generator.output_path
  source_code_hash = data.archive_file.index_generator.output_base64sha256

  environment {
    variables = {
      BUCKET_NAME = aws_s3_bucket.bags.id
    }
  }
}

resource "aws_lambda_permission" "allow_s3" {
  statement_id  = "AllowS3Invoke"
  action        = "lambda:InvokeFunction"
  function_name = aws_lambda_function.index_generator.function_name
  principal     = "s3.amazonaws.com"
  source_arn    = aws_s3_bucket.bags.arn
}

# --- Outputs ---

output "bucket_name" {
  value = aws_s3_bucket.bags.id
}

output "bucket_url" {
  value = "https://${aws_s3_bucket.bags.bucket}.s3.amazonaws.com"
}
